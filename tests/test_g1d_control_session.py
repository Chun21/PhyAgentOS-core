import time
from dataclasses import replace
from types import SimpleNamespace

import pytest

from PhyAgentOS.skill_runtime.g1d_adapter import SafetyFaultError, make_lowstate_frame
from PhyAgentOS.skill_runtime.g1d_control_session import ControlSession
from PhyAgentOS.skill_runtime.g1d_executor import StreamPhase, StreamSample


class Source:
    def __init__(self):
        self.tick = 0
        self.stale = False

    def read(self):
        self.tick += 1
        return make_lowstate_frame(mode_machine=5, tick=self.tick, positions=[.1]*35), time.monotonic() - (1 if self.stale else 0)


class Motion:
    def __init__(self):
        self.mode = "ai"
        self.releases = 0
        self.restores = 0

    def check(self):
        return self.mode

    def release(self):
        self.releases += 1
        self.mode = ""

    def restore(self):
        self.restores += 1
        self.mode = "ai"


class Sink:
    writer_guid = "self"
    matched = 1

    def __init__(self):
        self.samples = []

    def write(self, sample):
        self.samples.append(sample)


class Discovery:
    def __init__(self):
        self.other = set()

    def poll(self):
        return {"self"} | self.other


def make_session(tmp_path):
    return ControlSession(config={"approved_mode": 5, "gains": [[20, 1]]*35,
        "modes": [1]*35, "max_arm_excursion_rad": .15}, source=Source(), sink=Sink(),
        motion=Motion(), discovery=Discovery(), lock_path=tmp_path/"controller.lock")


def wait_for(predicate):
    deadline = time.monotonic() + 3
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(.01)


def test_start_holds_nonzero_pose_and_close_restores(tmp_path):
    session = make_session(tmp_path)
    try:
        session.start()
        wait_for(lambda: len(session.sink.samples) > 3)
        assert session.motion.releases == 1
        assert all(m.q == .1 for m in session.sink.samples[0].frame.motor_cmd)
        frame = session._last_frame
        commands = list(frame.motor_cmd)
        commands[12] = replace(commands[12], q=.8)
        commands[15] = replace(commands[15], q=.12)
        session.write(StreamSample(0, time.monotonic(), replace(frame, motor_cmd=tuple(commands)), StreamPhase.HOLD))
        assert session.sink.samples[-1].frame.motor_cmd[12].q == .1
        assert session.sink.samples[-1].frame.motor_cmd[15].q == .12
    finally:
        session.close()
    assert session.recovery == "ai_confirmed"
    count = len(session.sink.samples)
    time.sleep(.02)
    assert len(session.sink.samples) == count


def test_already_released_refuses_without_writes_or_restore(tmp_path):
    session = make_session(tmp_path)
    session.motion.mode = ""
    with pytest.raises(SafetyFaultError, match="expected factory"):
        session.start()
    assert not session.sink.samples and session.motion.restores == 0


def test_stale_feedback_stops_and_restores(tmp_path):
    session = make_session(tmp_path)
    try:
        session.start()
        session.source.stale = True
        wait_for(lambda: session.recovery == "ai_confirmed")
        assert not session.status()["ready"]
        with pytest.raises(SafetyFaultError):
            session.require()
    finally:
        session.close()


def test_competing_writer_prevents_override_on_recovery(tmp_path):
    session = make_session(tmp_path)
    try:
        session.start()
        session.discovery.other.add("competitor")
        wait_for(lambda: session.recovery.startswith("unconfirmed"))
        assert session.motion.restores == 0
    finally:
        session.close()


def test_plan_excursion_rejected_before_write(tmp_path):
    session = make_session(tmp_path)
    try:
        session.start()
        plan = SimpleNamespace(joint_solution=SimpleNamespace(left_q=[.5]*7, right_q=[.1]*7))
        with pytest.raises(SafetyFaultError, match="excursion"):
            session.validate_plan(plan)
    finally:
        session.close()


def test_intermediate_path_excursion_rejected_even_when_endpoint_is_safe(tmp_path):
    session = make_session(tmp_path)
    try:
        session.start()
        plan = SimpleNamespace(joint_solution=SimpleNamespace(left_q=[.1]*7, right_q=[.1]*7),
            joint_path=SimpleNamespace(points=([.1]*14, [.5]*14, [.1]*14)))
        with pytest.raises(SafetyFaultError, match="excursion"):
            session.validate_plan(plan)
    finally:
        session.close()


@pytest.mark.parametrize("value", [0, -1, 1.51, float("nan"), float("inf")])
def test_session_excursion_override_is_bounded(tmp_path, value):
    from PhyAgentOS.skill_runtime.g1d_control_session import validate_control_profile

    config = dict(make_session(tmp_path).config)
    config["max_arm_excursion_rad"] = 1.5
    validate_control_profile(config)
    config["max_arm_excursion_rad"] = value
    with pytest.raises(ValueError, match="excursion"):
        validate_control_profile(config)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -.5])
def test_invalid_commands_never_reach_sink(tmp_path, value):
    session = make_session(tmp_path)
    try:
        session.start()
        commands = list(session._last_frame.motor_cmd)
        commands[15] = replace(commands[15], q=value)
        frame = replace(session._last_frame, motor_cmd=tuple(commands))
        with pytest.raises(SafetyFaultError):
            session.write(StreamSample(0, time.monotonic(), frame, StreamPhase.HOLD))
        assert all(sample.frame.motor_cmd[15].q == .1 for sample in session.sink.samples)
    finally:
        session.close()


def test_native_hold_wire_frame_and_discovery_on_loopback(tmp_path, monkeypatch):
    monkeypatch.setenv("CYCLONEDDS_URI", '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic

    from PhyAgentOS.skill_runtime.g1d_bridge import CycloneLowCmdSink, _cyclone_types
    from PhyAgentOS.skill_runtime.g1d_control_session import LowCmdDiscovery

    participant = DomainParticipant(204)
    reader = DataReader(participant, Topic(participant, "rt/lowcmd", _cyclone_types()["LowCmd"]))
    session = make_session(tmp_path)
    session.sink = CycloneLowCmdSink(domain_id=204)
    session.discovery = LowCmdDiscovery(204)
    try:
        session.start()
        time.sleep(.02)
        samples = reader.take(32)
        assert samples
        assert all(abs(motor.q - .1) < 1e-6 for motor in samples[-1].motor_cmd)
        assert samples[-1].mode_machine == 5
        assert samples[-1].crc != 0
        assert session.sent > 0
    finally:
        session.close()
    assert session.recovery == "ai_confirmed"
