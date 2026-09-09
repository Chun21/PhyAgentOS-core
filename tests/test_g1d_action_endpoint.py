"""Exercise the packaged upstream Forge handler and production Action binding."""

import asyncio
from pathlib import Path

from test_g1d_executor import Harness, make_executor, run_to_completion


def test_packaged_action_endpoint_identity_stop_and_results(monkeypatch):
    artifact = Path(__file__).parents[1] / "bundles/g1d-manipulation/artifacts/g1d-runtime"
    monkeypatch.syspath_prepend(str(artifact))
    from forge_tool import ToolContext, ToolExecutionKey, ToolRequest

    from PhyAgentOS.skill_runtime.g1d_action_endpoint import G1DActionEndpoint

    h = Harness()
    h.publish_state()
    executor = make_executor(h)
    endpoint = G1DActionEndpoint(executor, lambda request, context: None)

    class Events:
        async def emit(self, event):
            pass

    async def run():
        key = ToolExecutionKey("gateway-1", "attempt-1")
        context = ToolContext(
            key, "g1d.dual_arm.execute_pose", "test", "g1d.dual_arm", "execute_pose", caller_id="a"
        )
        arguments = {"plan_id": h.make_plan().plan_id}
        accepted = await endpoint.start(ToolRequest(arguments), context, Events())
        assert accepted.details["invocation_id"] == "gateway-1"
        assert executor.get_invocation("gateway-1").caller_id == "a"
        assert (await endpoint.status(key)).phase == "accepted"
        assert (await endpoint.result(key)).status == "pending"
        run_to_completion(h)
        assert (await endpoint.result(key)).result.status == "succeeded"
        assert (await endpoint.status(key)).phase == "completed"
        retry_key = ToolExecutionKey("gateway-2", "attempt-2")
        retry = ToolContext(
            retry_key,
            "g1d.dual_arm.execute_pose",
            "test",
            "g1d.dual_arm",
            "execute_pose",
            caller_id="a",
        )
        assert (await endpoint.start(ToolRequest(arguments), retry, Events())).details[
            "invocation_id"
        ] == "gateway-1"
        assert (await endpoint.result(retry_key)).result.status == "succeeded"
        bad = ToolExecutionKey("gateway-1", "wrong-attempt")
        assert (await endpoint.result(bad)).status == "not_found"

        from forge_tool.wire import ToolProtocolError

        class AlreadyObserved:
            async def emit(self, event):
                raise ToolProtocolError("FORGE_ENDPOINT_EVENT_AFTER_TERMINAL", "queried first")

        endpoint._events[key] = AlreadyObserved()
        await endpoint.emit_updates()
        assert not endpoint._events
        assert (await endpoint.result(key)).result.status == "succeeded"

    asyncio.run(run())
    executor.close()


def test_production_gateway_executes_and_returns_observed_result(tmp_path, monkeypatch):
    import time

    from fastapi.testclient import TestClient

    from PhyAgentOS.skill_runtime.g1d_adapter import make_lowstate_frame
    from PhyAgentOS.skill_runtime.g1d_execution_runtime import G1DExecutionRuntime
    from PhyAgentOS.skill_runtime.g1d_executor import RecordingSink

    artifact = Path(__file__).parents[1] / "bundles/g1d-manipulation/artifacts/g1d-runtime"
    monkeypatch.syspath_prepend(str(artifact))
    from PhyAgentOS.skill_runtime.g1d_gateway import G1DGateway

    class Source:
        def __init__(self):
            self.tick = 0

        def read(self):
            self.tick += 1
            return make_lowstate_frame(
                mode_machine=7, tick=self.tick, positions=[0.0] * 35
            ), time.monotonic()

    runtime = G1DExecutionRuntime(
        artifact.parents[1],
        source=Source(),
        sink=RecordingSink(),
        clock=time.monotonic,
        journal_path=tmp_path / "actions.sqlite",
        approved_mode=7,
        gains=[(20.0, 1.0)] * 35,
        modes=[1] * 35,
        require_ownership=lambda: None,
    )
    runtime.poll()
    poses = runtime.kinematics.solve_fk([0.0] * 7, [0.0] * 7)
    targets = {
        side: {
            "frame_id": "unirobot_g1d_fixed_base",
            "position_m": list(p.position_m),
            "orientation_xyzw": list(p.orientation_xyzw),
        }
        for side, p in zip(("left", "right"), poses)
    }
    with TestClient(G1DGateway(runtime).app()) as client:
        response = client.post("/tools/g1d.dual_arm/plan_pose:invoke", json={"arguments": targets})
        plan = response.json()["data"]["response"]["result"]["outputs"]
        response = client.post(
            "/tools/g1d.dual_arm/execute_pose:invoke",
            json={"arguments": {"plan_id": plan["plan_id"], "caller_id": "a"}},
        )
        assert response.status_code == 202, response.text
        identity = response.json()["data"]["invocation_id"]
        assert response.json()["data"]["deadline_ms"] > int(time.time() * 1000) + 25000
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            response = client.get(f"/invocations/{identity}/result")
            data = response.json()
            if data.get("data", {}).get("status") == "available":
                break
            time.sleep(0.03)
        record = runtime.executor.active_invocation()
        assert record.status.value in ("succeeded", "unknown"), data
        if record.status.value == "unknown":
            assert record.stop_reason == "watchdog_missed_cycles"
            assert data["data"]["result"]["status"] == "unknown"
        else:
            assert data["data"]["result"]["status"] == "succeeded"
        assert client.get(f"/g1d/evidence/{identity}").status_code == 200
