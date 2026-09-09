"""Use real Agent binding and HTTP contracts with measured-state simulation."""

import asyncio
import json
import time
from pathlib import Path
from types import SimpleNamespace

import httpx
import pytest


@pytest.mark.parametrize("wave", [False, True])
def test_agent_activation_bound_task_out_and_back(tmp_path, monkeypatch, wave):
    import PhyAgentOS.skill_runtime  # noqa: F401 - load source package before zipapp path

    bundle = Path(__file__).parents[1] / "bundles/g1d-manipulation"
    monkeypatch.syspath_prepend(str(bundle / "artifacts/g1d-runtime"))
    from fastapi.testclient import TestClient

    from PhyAgentOS.forge.tool_client import ForgeToolClient
    from PhyAgentOS.skill_runtime.g1d_adapter import make_lowstate_frame
    from PhyAgentOS.skill_runtime.g1d_execution_runtime import G1DExecutionRuntime
    from PhyAgentOS.skill_runtime.g1d_executor import RecordingSink
    from PhyAgentOS.skill_runtime.g1d_gateway import G1DGateway
    from scripts import g1d_agent_smoke

    sink = RecordingSink()

    simulated_time = [time.monotonic()]

    class Source:
        tick = 0

        def read(self):
            self.tick += 1
            simulated_time[0] += .002
            q = [0.0] * 35
            q[15:29] = [.25, -.01, .01, 1.10, .006, .29, .009,
                        .23, -.007, .078, 1.26, .021, -.065, -.011]
            if sink.samples:
                q = [motor.q for motor in sink.samples[-1].frame.motor_cmd]
            return make_lowstate_frame(mode_machine=5, tick=self.tick, positions=q), simulated_time[0]

    runtime = G1DExecutionRuntime(bundle, source=Source(), sink=sink,
        journal_path=tmp_path / "executor.sqlite", approved_mode=5,
        gains=[(20, 1)] * 35, modes=[1] * 35, require_ownership=lambda: None,
        clock=lambda: simulated_time[0])
    gateway = G1DGateway(runtime)
    app = gateway.app()
    monkeypatch.setattr(g1d_agent_smoke, "ForgeToolClient",
        lambda url: ForgeToolClient(url, transport=httpx.ASGITransport(app=app)))
    args = SimpleNamespace(workspace=tmp_path / "agent", skills_root=bundle.parent,
        gateway="http://testserver", execute=True, wave=wave)
    with TestClient(app):
        try:
            asyncio.run(g1d_agent_smoke.run(args))
        except Exception:
            if gateway.worker_error:
                raise gateway.worker_error
            raise
    reports = list(args.workspace.glob("task_*-motion.json"))
    assert len(reports) == 1
    report = json.loads(reports[0].read_text())
    assert report["accepted"], report
    assert report["agent_task_status"] == "succeeded"
    assert sink.samples
    if wave:
        assert report["wrist_range_rad"] > .6
        assert report["right_lift_m"] > .3
        assert report["peak_right_height_m"] >= 1.0
        motion = [i for i, sample in enumerate(sink.samples) if sample.phase.value == "motion"]
        assert motion == list(range(motion[0], motion[-1] + 1))
        assert all(abs(sample.frame.motor_cmd[28].dq) <= .5 + 1e-9 for sample in sink.samples)
