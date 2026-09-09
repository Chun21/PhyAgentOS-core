"""Tests for the self-contained rt/lowstate <-> rt/lowcmd DDS bridge.

The bridge owns its IDL dataclasses (mirroring the Unitree HG wire layout,
typenames match the robot's native topics) and wraps cyclonedds.  All
cyclonedds interactions are injectable, so repository tests need no DDS
runtime.  The HG wire CRC is a pure-python replication of the robot firmware
algorithm (MSB-first 0x04C11DB7 over the packed words).
"""

from __future__ import annotations

import pytest

from PhyAgentOS.skill_runtime.g1d_adapter import G1DAdapter
from PhyAgentOS.skill_runtime.g1d_bridge import (
    BridgeCRCError,
    BridgeUnavailableError,
    CycloneLowCmdSink,
    CycloneLowStateSource,
    HGLowCmd,
    HGLowState,
    HGMotorCmd,
    convert_hg_lowcmd,
    convert_hg_lowstate,
    hg_lowcmd_crc,
    hg_lowstate_crc,
    hg_lowstate_pack_size,
    lowcmd_from_frame,
    lowcmd_to_wire,
)


# ---------------------------------------------------------------------------
# IDL dataclasses: wire layout
# ---------------------------------------------------------------------------


def test_idl_typenames_match_unitree_hg_topics() -> None:
    # The robot's DDS participants match on the XTypes type name; these must
    # equal the unitree_hg names byte for byte.
    assert HGLowState.__idl_typename__ == "unitree_hg.msg.dds_.LowState_"
    assert HGLowCmd.__idl_typename__ == "unitree_hg.msg.dds_.LowCmd_"


def test_hg_lowstate_wire_layout() -> None:
    assert hg_lowstate_pack_size() == 2092
    state = HGLowState()
    assert len(state.motor_state) == 35
    assert len(state.version) == 2
    assert len(state.wireless_remote) == 40
    assert len(state.reserve) == 4


def test_hg_lowcmd_wire_layout() -> None:
    cmd = HGLowCmd()
    assert len(cmd.motor_cmd) == 35
    assert len(cmd.reserve) == 4
    assert cmd.mode_pr == 0 and cmd.mode_machine == 0 and cmd.crc == 0
    assert HGMotorCmd().mode == 0


# ---------------------------------------------------------------------------
# HG CRC (pure python replication of firmware algorithm)
# ---------------------------------------------------------------------------


def make_state(**overrides) -> HGLowState:
    state = HGLowState()
    state.mode_machine = overrides.get("mode_machine", 7)
    state.tick = overrides.get("tick", 1000)
    for index in range(35):
        state.motor_state[index].mode = 1
        state.motor_state[index].q = 0.05 * index
    for key, value in overrides.items():
        setattr(state, key, value)
    state.crc = hg_lowstate_crc(state)
    return state


def test_hg_crc_matches_firmware_bitwise_oracle() -> None:
    import random

    from PhyAgentOS.skill_runtime.g1d_bridge import _crc32_msb

    def oracle(words):
        crc = 0xFFFFFFFF
        for word in words:
            crc ^= word
            for _ in range(32):
                crc = ((crc << 1) ^ (0x04C11DB7 if crc & 0x80000000 else 0)) & 0xFFFFFFFF
        return crc

    randomizer = random.Random(102)
    for words in ([], [0], [0xFFFFFFFF], [0x12345678],
                  [randomizer.getrandbits(32) for _ in range(522)]):
        assert _crc32_msb(words) == oracle(words)


def test_hg_lowstate_crc_is_deterministic_and_tamper_sensitive() -> None:
    state = make_state()
    assert hg_lowstate_crc(state) == state.crc
    assert state.crc != 0

    tampered = make_state()
    tampered.motor_state[20].q += 0.001
    assert hg_lowstate_crc(tampered) != tampered.crc

    assert hg_lowstate_crc(make_state(tick=1001)) == make_state(tick=1001).crc


def test_hg_lowstate_crc_rejects_structurally_invalid_state() -> None:
    state = make_state()
    state.motor_state = state.motor_state[:34]  # wrong count
    with pytest.raises(ValueError, match="35"):
        hg_lowstate_crc(state)


# ---------------------------------------------------------------------------
# Conversion: HG IDL <-> transport-neutral frames
# ---------------------------------------------------------------------------


def test_convert_hg_lowstate_maps_fields_and_revalidates_crc() -> None:
    state = make_state(mode_machine=7, tick=4242)

    frame = convert_hg_lowstate(state, received_at=10.0)

    assert frame.mode_machine == 7
    assert frame.tick == 4242
    assert len(frame.positions) == 35
    assert frame.positions[20] == pytest.approx(0.05 * 20)
    assert frame.modes[20] == 1
    assert frame.crc == frame.calculated_crc()  # neutral-frame re-sign

    clock = [10.0]
    adapter = G1DAdapter(clock=lambda: clock[0], max_state_age_s=0.1,
                         approved_mode_machine=7)
    adapter.ingest(frame, received_at=10.0)
    view = adapter.query_state()
    assert tuple(s.q for s in view.left_arm) == tuple(0.05 * i for i in range(15, 22))
    assert tuple(s.q for s in view.right_arm) == tuple(0.05 * i for i in range(22, 29))


def test_convert_hg_lowstate_rejects_wire_crc_mismatch() -> None:
    state = make_state()
    state.motor_state[18].q = 0.999  # tamper after signing
    with pytest.raises(BridgeCRCError, match="rt/lowstate"):
        convert_hg_lowstate(state, received_at=10.0)


def test_lowcmd_to_wire_round_trips_through_pack() -> None:
    # The packed struct is exactly the HG LowCmd_ wire layout.
    cmd = HGLowCmd(mode_machine=7)
    cmd.motor_cmd[15].q = 0.123
    cmd.motor_cmd[22].kp = 40.0
    cmd.crc = lowcmd_to_wire(cmd)[1]

    packed = lowcmd_to_wire(cmd)[0]
    assert len(packed) == 1004  # 4 + 35*28 + 16 + 4 (SDK HGLowCmd pack size)
    assert packed[0] == cmd.mode_pr
    assert packed[1] == cmd.mode_machine


def test_convert_hg_lowcmd_rejects_crc_mismatch() -> None:
    cmd = HGLowCmd(mode_machine=7)
    cmd.crc = 12345  # wrong
    with pytest.raises(BridgeCRCError, match="rt/lowcmd"):
        convert_hg_lowcmd(cmd)


# ---------------------------------------------------------------------------
# CycloneLowStateSource with a fake cyclonedds
# ---------------------------------------------------------------------------


class FakeDataReader:
    def __init__(self, subscriber, topic):
        self.subscriber = subscriber
        self.topic = topic
        self.samples: list = []

    def take(self):
        taken, self.samples = self.samples, []
        return taken


class FakeCyclone:
    """Records cyclonedds calls; fake samples delivered on demand."""

    def __init__(self) -> None:
        self.records: list = []
        self.reader: FakeDataReader | None = None

    class FakeDomainParticipant:
        def __init__(self, records, domain_id):
            records.append(("participant", domain_id))

    class FakeTopic:
        def __init__(self, records, name, dtype):
            records.append(("topic", name, dtype.__idl_typename__))
            self.name = name

    class FakeSubscriber:
        def __init__(self, records, dp):
            records.append(("subscriber",))

    def DomainParticipant(self, domain_id=0):
        return self.FakeDomainParticipant(self.records, domain_id)

    def Topic(self, dp, name, dtype):
        return self.FakeTopic(self.records, name, dtype)

    def Subscriber(self, dp):
        return self.FakeSubscriber(self.records, dp)

    def DataReader(self, subscriber, topic):
        self.reader = FakeDataReader(subscriber, topic)
        return self.reader


def test_cyclone_source_subscribes_and_converts_latest_frame() -> None:
    cyclone = FakeCyclone()
    source = CycloneLowStateSource(cyclone=cyclone, domain_id=0)

    assert ("participant", 0) in cyclone.records
    assert ("topic", "rt/lowstate", "unitree_hg.msg.dds_.LowState_") in cyclone.records

    assert source.read() is None  # nothing yet

    cyclone.reader.samples.append(make_state(tick=7))
    frame, received_at = source.read()
    assert frame.tick == 7
    assert frame.crc == frame.calculated_crc()
    assert received_at > 0

    # Newer frames replace older ones; older frames are dropped.
    cyclone.reader.samples.append(make_state(tick=6))
    cyclone.reader.samples.append(make_state(tick=8))
    assert source.read()[0].tick == 8


def test_cyclone_source_reports_crc_invalid_wire_frames() -> None:
    cyclone = FakeCyclone()
    source = CycloneLowStateSource(cyclone=cyclone)

    broken = make_state()
    broken.motor_state[3].q += 1.0  # crc now stale
    cyclone.reader.samples.append(broken)
    with pytest.raises(BridgeCRCError):
        source.read()

    good = make_state(tick=9)
    cyclone.reader.samples.append(good)
    assert source.read()[0].tick == 9


def test_cyclone_source_missing_dependency_is_explicit(monkeypatch) -> None:
    import sys

    monkeypatch.setitem(sys.modules, "cyclonedds", None)
    with pytest.raises(BridgeUnavailableError, match="cyclonedds"):
        CycloneLowStateSource(cyclone=None)


# ---------------------------------------------------------------------------
# CycloneLowCmdSink with a fake cyclonedds publisher
# ---------------------------------------------------------------------------


class FakeDataWriter:
    def __init__(self):
        self.written: list = []

    def write(self, message):
        self.written.append(message)


class FakePubCyclone(FakeCyclone):
    def __init__(self) -> None:
        super().__init__()
        self.writer = FakeDataWriter()

    class FakePublisher:
        def __init__(self, records):
            records.append(("publisher",))

    def Publisher(self, dp):
        return self.FakePublisher(self.records)

    def DataWriter(self, publisher, topic):
        self.records.append(("writer", topic.name))
        return self.writer


def make_cmd_frame() -> "LowCmdFrame":
    from PhyAgentOS.skill_runtime.g1d_adapter import G1DAdapter, make_lowstate_frame

    adapter = G1DAdapter(clock=lambda: 1.0, approved_mode_machine=7)
    state = make_lowstate_frame(mode_machine=7, tick=1, positions=[0.0] * 35,
                                mode=[1] * 35)
    adapter.ingest(state, received_at=1.0)
    return adapter.hold_frame()


def test_cyclone_lowcmd_sink_publishes_signed_wire_frames() -> None:
    from types import SimpleNamespace

    cyclone = FakePubCyclone()
    sink = CycloneLowCmdSink(cyclone=cyclone)

    assert ("topic", "rt/lowcmd", "unitree_hg.msg.dds_.LowCmd_") in cyclone.records

    frame = make_cmd_frame()
    sink.write(SimpleNamespace(frame=frame))

    assert len(cyclone.writer.written) == 1
    published = cyclone.writer.written[0]
    # Published frame is wire-signed and CRC-valid.
    assert published.crc != 0
    assert hg_lowcmd_crc(published) == published.crc
    assert published.mode_machine == frame.mode_machine
    assert published.motor_cmd[15].q == frame.motor_cmd[15].q


def test_cyclone_lowcmd_sink_skips_non_frame_samples() -> None:
    from types import SimpleNamespace

    cyclone = FakePubCyclone()
    sink = CycloneLowCmdSink(cyclone=cyclone)

    sink.write(SimpleNamespace(command="dex1"))  # Dex1 sample: no lowcmd frame
    assert cyclone.writer.written == []


def test_lowcmd_from_frame_round_trips_through_convert() -> None:
    frame = make_cmd_frame()
    cmd = lowcmd_from_frame(frame)
    assert cmd.crc == hg_lowcmd_crc(cmd)
    # Neutral round trip preserves the command payload.
    back = convert_hg_lowcmd(cmd)
    assert back.mode_machine == frame.mode_machine
    assert back.motor_cmd[22].q == frame.motor_cmd[22].q
    assert back.motor_cmd[15].kp == frame.motor_cmd[15].kp
