"""Run the shipped zipapp through the real Forge HTTP Gateway and loopback DDS."""

import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import asdict, replace
from pathlib import Path

import httpx
import pytest
from jsonschema import validate

from PhyAgentOS.skill_runtime.archive import sha256_file
from PhyAgentOS.skill_runtime.g1d_bridge import HGLowState, _cyclone_types, hg_lowstate_crc
from PhyAgentOS.skill_runtime.installer import NodeInstaller, SkillInstaller
from PhyAgentOS.skill_runtime.runtime_manifest import normalize_arch
from PhyAgentOS.skill_runtime.state import RuntimeStateStore
from scripts.package_skill import package

BUNDLE = Path(__file__).parents[1] / "bundles/g1d-manipulation"


@pytest.mark.parametrize("launch_mode", ["direct", "dora"])
def test_installed_gateway_state_plan_actions_and_no_lowcmd(tmp_path, monkeypatch, launch_mode):
    # Confine the test participant and child process to loopback on a non-robot domain.
    cyclone_xml = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
    monkeypatch.setenv("CYCLONEDDS_URI", cyclone_xml)
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic

    archive = package(BUNDLE, tmp_path / "archives")
    store = RuntimeStateStore(tmp_path / "state")
    manifest = SkillInstaller(tmp_path / "skills", state_store=store).install(
        archive, expected_sha256=sha256_file(archive)
    )
    # The zipapp itself is arch-neutral; this host exercise uses host numerical wheels.
    # The shipped aarch64 lock is checked separately and never relabelled in the Bundle.
    lock = replace(manifest.artifacts.nodes["g1d-runtime"], arch=normalize_arch())
    node = NodeInstaller(tmp_path / "nodes", state_store=store).install(
        manifest.bundle_root / "artifacts/g1d-runtime-0.2.0.tar.gz", lock
    )
    with socket.socket() as available:
        available.bind(("127.0.0.1", 0))
        port = available.getsockname()[1]
    domain = 170 + os.getpid() % 30
    participant = DomainParticipant(domain)
    types = _cyclone_types()
    assert types is not None
    writer = DataWriter(participant, Topic(participant, "rt/lowstate", types["LowState"]))
    commands = DataReader(participant, Topic(participant, "rt/lowcmd", types["LowCmd"]))
    stop = threading.Event()
    mode = ["fresh"]

    def publish():
        tick = 1
        while not stop.wait(0.02):
            if mode[0] == "silent":
                continue
            state = HGLowState(mode_machine=7, tick=tick)
            tick += 1
            state.crc = hg_lowstate_crc(state)
            if mode[0] == "corrupt":
                state.crc ^= 1
            payload = asdict(state)
            payload["imu_state"] = types["IMUState"](**payload["imu_state"])
            payload["motor_state"] = [
                types["MotorState"](**motor) for motor in payload["motor_state"]
            ]
            writer.write(types["LowState"](**payload))

    env = {
        **os.environ,
        "PYTHONPATH": "",
        "PYTHONNOUSERSITE": "1",
        "PAOS_G1D_DDS_DOMAIN": str(domain),
        "CYCLONEDDS_URI": cyclone_xml,
    }
    env.pop("PAOS_SKILL_ROOT", None)
    env.pop("PAOS_G1D_RUNTIME_IMPL", None)
    output = (tmp_path / "runtime.log").open("w+")
    command = [sys.executable, "-I", str(node), "--port", str(port)]
    if launch_mode == "dora":
        import yaml

        dora = shutil.which("dora")
        if dora is None:
            pytest.fail("installed-runtime acceptance requires Dora CLI 0.4.1")
        flow = tmp_path / "dataflow.yaml"
        flow.write_text(
            yaml.safe_dump(
                {
                    "nodes": [
                        {
                            "id": "g1d-runtime",
                            "path": str(node),
                            "args": f"--port {port}",
                            "inputs": {"lifecycle_tick": "dora/timer/millis/100"},
                        }
                    ]
                }
            )
        )
        command = [dora, "run", str(flow)]
        env["PATH"] = str(Path(sys.executable).parent) + os.pathsep + env["PATH"]
    process = subprocess.Popen(
        command,
        cwd=tmp_path,
        env=env,
        stdout=output,
        stderr=output,
        start_new_session=True,
    )
    publisher = threading.Thread(target=publish, daemon=True)
    client = httpx.Client(base_url=f"http://127.0.0.1:{port}", timeout=2, trust_env=False)

    def until(fn):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if process.poll() is not None:
                output.seek(0)
                pytest.fail(output.read())
            try:
                result = fn()
                if result:
                    return result
            except httpx.TransportError:
                pass
            time.sleep(0.02)
        output.seek(0)
        pytest.fail("Gateway condition timed out:\n" + output.read())

    def query(operation, arguments):
        return client.post(
            f"/tools/g1d.dual_arm/{operation}:invoke", json={"arguments": arguments}
        ).json()["data"]["response"]["result"]

    try:
        discovery = until(lambda: client.get("/tools").json())
        assert len(discovery["data"]["tools"]) == 4
        assert query("state", {})["outputs"]["safety_gate"] == "state_unavailable"
        publisher.start()
        state = until(
            lambda: (
                value
                if (value := query("state", {})["outputs"]).get("safety_gate")
                == "read_only_unverified"
                else None
            )
        )
        schemas = json.loads((BUNDLE / "tools/tools.json").read_text())
        validate(state, schemas["g1d.dual_arm.state"]["output_schema"])
        assert state["mode_machine"] == 7 and state["state_age_ms"] <= 100
        assert state["left_arm"]["joint_positions_rad"] == [0] * 7
        assert state["right_arm"]["joint_positions_rad"] == [0] * 7
        assert state["action_ready"] is False
        # Independent reference fixture from the URDF chain, not child-process FK.
        targets = {
            "left": {
                "frame_id": "g1d_base",
                "position_m": [0.192172736674722, 0.189179704321410, 0.001816614310073],
                "orientation_xyzw": [
                    0.154489414151274,
                    0.168929134576250,
                    0.006023824933432,
                    0.973426772767057,
                ],
            },
            "right": {
                "frame_id": "g1d_base",
                "position_m": [0.283835956813965, -0.133142519604804, 0.078952906599031],
                "orientation_xyzw": [
                    -0.127835655672707,
                    0.066340872415173,
                    0.010632934227831,
                    0.989516990503766,
                ],
            },
        }
        plan = query("plan_pose", targets)
        assert plan["status"] == "succeeded", plan
        validate(plan["outputs"], schemas["g1d.dual_arm.plan_pose"]["output_schema"])
        rejected = client.post(
            "/tools/g1d.dual_arm/plan_pose:invoke",
            json={"arguments": {**targets, "raw_command": []}},
        )
        assert rejected.status_code == 422
        assert rejected.json()["error"]["code"] == "invalid_arguments"
        for operation, args, code in (
            (
                "execute_pose",
                {"plan_id": plan["outputs"]["plan_id"], "caller_id": "test"},
                "action_not_ready",
            ),
            ("stop", {}, "no_active_operation"),
        ):
            admission = client.post(
                f"/tools/g1d.dual_arm.{operation}:invoke", json={"arguments": args}
            ).json()["data"]
            invocation = admission["invocation_id"]
            terminal = until(
                lambda: (
                    v
                    if (v := client.get(f"/invocations/{invocation}").json()["data"])["phase"]
                    == "failed"
                    else None
                )
            )
            assert terminal["error"]["code"] == code
        mode[0] = "corrupt"
        until(lambda: query("state", {})["outputs"]["safety_gate"] == "state_corrupt")
        assert query("plan_pose", targets)["status"] == "failed"
        mode[0] = "fresh"
        until(lambda: query("state", {})["outputs"]["safety_gate"] == "read_only_unverified")
        mode[0] = "silent"
        until(lambda: query("state", {})["outputs"]["safety_gate"] == "state_stale")
        assert query("plan_pose", targets)["status"] == "failed"
        assert commands.take() == []
    finally:
        stop.set()
        if publisher.is_alive():
            publisher.join(2)
        client.close()
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)
        output.close()
    assert process.returncode in (0, -signal.SIGINT)
