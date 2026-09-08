"""Real DDS, confined to loopback; never connects to the robot domain."""

import os
import time

import pytest


def test_dex1_go_wire_opening_direction_and_faults(monkeypatch):
    monkeypatch.setenv(
        "CYCLONEDDS_URI",
        '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>',
    )
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic

    from PhyAgentOS.skill_runtime.g1d_dex1 import Dex1CommandSample, Dex1Integration
    from PhyAgentOS.skill_runtime.g1d_dex1_bridge import (
        CycloneDex1Bridge,
        DexMotorCmds,
        DexMotorState,
        DexMotorStates,
    )

    domain = 205 + os.getpid() % 15
    participant = DomainParticipant(domain)
    reader = DataReader(participant, Topic(participant, "rt/dex1/left/cmd", DexMotorCmds))
    writer = DataWriter(participant, Topic(participant, "rt/dex1/left/state", DexMotorStates))
    bridge = CycloneDex1Bridge(domain_id=domain, verified_open_q=5.4, verified_closed_q=0.0)
    integration = Dex1Integration(clock=time.monotonic, sources={"left": bridge})
    end = time.monotonic() + 3
    received = []
    while time.monotonic() < end and not received:
        bridge.write(Dex1CommandSample(time.monotonic(), integration.command("left", 1.0)))
        time.sleep(0.01)
        received = reader.take(1)
    assert received and received[0].cmds[0].q == pytest.approx(5.4)
    msg = DexMotorStates([DexMotorState(1, 2.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20, 0, [0, 0])])
    end = time.monotonic() + 3
    while time.monotonic() < end:
        writer.write(msg)
        time.sleep(0.01)
        state = integration.readiness("left")
        if state.ok:
            break
    assert state.ok and state.opening == pytest.approx(0.5)
    assert not integration.readiness("right").ok
    time.sleep(0.11)
    assert not integration.readiness("left").ok
