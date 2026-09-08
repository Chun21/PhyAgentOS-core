"""Self-contained rt/lowstate <-> rt/lowcmd DDS bridge for the Unitree G1_D.

This module owns everything needed to talk to the robot's low-level DDS
topics:

- Plain-python message records (:class:`HGLowState`, :class:`HGLowCmd`, ...)
  mirroring the Unitree HG wire layout byte for byte, including the
  ``unitree_hg.msg.dds_`` XTypes type names so native robot participants
  match on the wire.
- A pure-python replication of the firmware CRC (MSB-first 0x04C11DB7 over
  the packed words, final word excluded), applied to both directions.
- Thin cyclonedds wrappers: :class:`CycloneLowStateSource` (subscribe
  ``rt/lowstate``) and :class:`CycloneLowCmdSink` (publish ``rt/lowcmd``,
  implementing the executor's command-sink protocol).

No Unitree SDK or UniRobot code is imported.  cyclonedds is imported lazily;
repository tests inject fakes and run without any DDS runtime.

NOTE: this module deliberately does NOT use ``from __future__ import
annotations`` — cyclonedds resolves IdlStruct class annotations as raw
objects, and string annotations break its type normalization.
"""

import struct
import time
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any

from PhyAgentOS.skill_runtime.g1d_adapter import (
    LowCmdFrame,
    LowStateFrame,
    MotorCommand,
    TOTAL_MOTOR_SLOTS,
)

DEFAULT_DOMAIN_ID = 0
STATE_TOPIC = "rt/lowstate"
CMD_TOPIC = "rt/lowcmd"

# cyclonedds IDL machinery, imported when the DDS runtime is available.  The
# names live at module level because cyclonedds resolves the (string) class
# annotations of IdlStruct types against the defining module's globals.
try:  # pragma: no cover - environment dependent
    import cyclonedds.idl as idl  # type: ignore[import-untyped]
    import cyclonedds.idl.annotations as annotate  # type: ignore[import-untyped]
    import cyclonedds.idl.types as types  # type: ignore[import-untyped]
    _HAVE_CYCLONEDDS = True
except ImportError:  # pragma: no cover - environment dependent
    idl = None
    annotate = None
    types = None
    _HAVE_CYCLONEDDS = False

# HG wire layouts, replicated from the Unitree HG IDL (see the research note
# docs/agents/research-g1d-lowcmd.md).  LowState_ packs to 2092 bytes,
# LowCmd_ to 1004 bytes; the firmware CRC runs over all words except the
# final CRC word.
_HG_LOWSTATE_FMT = "<2I2B2xI" + "13fh2x" + "B3x4f2hf7I" * TOTAL_MOTOR_SLOTS + "40B5I"
_HG_LOWCMD_FMT = "<2B2x" + "B3x5fI" * TOTAL_MOTOR_SLOTS + "5I"


class BridgeError(RuntimeError):
    """Base class for explicit robot-bridge errors."""


class BridgeCRCError(BridgeError):
    """A real rt/lowstate or rt/lowcmd frame failed its HG wire CRC check."""


class BridgeUnavailableError(BridgeError):
    """The DDS runtime (cyclonedds) is not importable in this environment."""


# ---------------------------------------------------------------------------
# Plain-python HG message records (wire-mirror of the unitree_hg IDL)
# ---------------------------------------------------------------------------


@dataclass
class HGIMUState:
    __idl_typename__ = "unitree_hg.msg.dds_.IMUState_"

    quaternion: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    gyroscope: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accelerometer: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    temperature: int = 25


@dataclass
class HGMotorState:
    __idl_typename__ = "unitree_hg.msg.dds_.MotorState_"

    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    ddq: float = 0.0
    tau_est: float = 0.0
    temperature: list[int] = field(default_factory=lambda: [0, 0])
    vol: float = 0.0
    sensor: list[int] = field(default_factory=lambda: [0, 0])
    motorstate: int = 0
    reserve: list[int] = field(default_factory=lambda: [0, 0, 0, 0])


@dataclass
class HGLowState:
    """unitree_hg.msg.dds_.LowState_ (35-slot HG robot state)."""

    __idl_typename__ = "unitree_hg.msg.dds_.LowState_"

    version: list[int] = field(default_factory=lambda: [1, 0])
    mode_pr: int = 0
    mode_machine: int = 0
    tick: int = 0
    imu_state: HGIMUState = field(default_factory=HGIMUState)
    motor_state: list[HGMotorState] = field(
        default_factory=lambda: [HGMotorState() for _ in range(TOTAL_MOTOR_SLOTS)]
    )
    wireless_remote: list[int] = field(default_factory=lambda: [0] * 40)
    reserve: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    crc: int = 0


@dataclass
class HGMotorCmd:
    __idl_typename__ = "unitree_hg.msg.dds_.MotorCmd_"

    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    reserve: int = 0


@dataclass
class HGLowCmd:
    """unitree_hg.msg.dds_.LowCmd_ (35-slot HG robot command)."""

    __idl_typename__ = "unitree_hg.msg.dds_.LowCmd_"

    mode_pr: int = 0
    mode_machine: int = 0
    motor_cmd: list[HGMotorCmd] = field(
        default_factory=lambda: [HGMotorCmd() for _ in range(TOTAL_MOTOR_SLOTS)]
    )
    reserve: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    crc: int = 0


# ---------------------------------------------------------------------------
# HG wire CRC (pure-python replication of the firmware crc32_core)
# ---------------------------------------------------------------------------


def _crc32_msb(words: Sequence[int]) -> int:
    crc = 0xFFFFFFFF
    polynomial = 0x04C11DB7
    for word in words:
        bit = 1 << 31
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) & 0xFFFFFFFF) ^ polynomial
            else:
                crc = (crc << 1) & 0xFFFFFFFF
            if word & bit:
                crc ^= polynomial
            bit >>= 1
    return crc


def _pack_lowstate(state: Any) -> bytearray:
    orig: list[Any] = [
        state.version[0], state.version[1],
        state.mode_pr, state.mode_machine, state.tick,
    ]
    imu = state.imu_state
    orig.extend(imu.quaternion)
    orig.extend(imu.gyroscope)
    orig.extend(imu.accelerometer)
    orig.extend(imu.rpy)
    orig.append(imu.temperature)
    for motor in state.motor_state:
        orig.extend(
            [motor.mode, motor.q, motor.dq, motor.ddq, motor.tau_est,
             *motor.temperature, motor.vol, *motor.sensor, motor.motorstate,
             *motor.reserve]
        )
    orig.extend(state.wireless_remote)
    orig.extend(state.reserve)
    orig.append(state.crc)
    return bytearray(struct.pack(_HG_LOWSTATE_FMT, *orig))


def hg_lowstate_pack_size() -> int:
    """Wire size of one HG LowState_ message (2092 bytes)."""

    return struct.calcsize(_HG_LOWSTATE_FMT)


def _require_slots(value: Sequence[Any], count: int) -> None:
    if len(value) != count:
        raise ValueError(f"motor_state must contain {count} entries")


def hg_lowstate_crc(state: Any) -> int:
    """CRC the firmware places in LowState_.crc, computed in pure python."""

    _require_slots(state.motor_state, TOTAL_MOTOR_SLOTS)
    packed = _pack_lowstate(state)
    words = [
        struct.unpack_from("<I", packed, offset)[0]
        for offset in range(0, len(packed) - 4, 4)
    ]
    return _crc32_msb(words)


def _pack_lowcmd(cmd: Any) -> bytearray:
    orig: list[Any] = [cmd.mode_pr, cmd.mode_machine]
    for motor in cmd.motor_cmd:
        orig.extend(
            [motor.mode, motor.q, motor.dq, motor.tau, motor.kp, motor.kd,
             motor.reserve]
        )
    orig.extend(cmd.reserve)
    orig.append(cmd.crc)
    return bytearray(struct.pack(_HG_LOWCMD_FMT, *orig))


def hg_lowcmd_crc(cmd: Any) -> int:
    """CRC the firmware places in LowCmd_.crc, computed in pure python."""

    _require_slots(cmd.motor_cmd, TOTAL_MOTOR_SLOTS)
    packed = _pack_lowcmd(cmd)
    words = [
        struct.unpack_from("<I", packed, offset)[0]
        for offset in range(0, len(packed) - 4, 4)
    ]
    return _crc32_msb(words)


def lowcmd_to_wire(cmd: Any) -> tuple[bytes, int]:
    """Sign an HGLowCmd: returns (packed wire bytes, computed CRC)."""

    crc = hg_lowcmd_crc(cmd)
    packed = _pack_lowcmd(cmd)
    struct.pack_into("<I", packed, len(packed) - 4, crc)
    return bytes(packed), crc


# ---------------------------------------------------------------------------
# Conversion: HG records <-> transport-neutral frames
# ---------------------------------------------------------------------------


def convert_hg_lowstate(state: Any, *, received_at: float) -> LowStateFrame:
    """Convert one HG LowState_ into the transport-neutral LowStateFrame.

    The real wire CRC is validated first; the neutral frame is then re-signed
    with its own CRC so the adapter's CRC gate checks conversion integrity.
    """

    if hg_lowstate_crc(state) != state.crc:
        raise BridgeCRCError(
            f"{STATE_TOPIC} frame failed its HG wire CRC check; refusing to ingest"
        )
    frame = LowStateFrame(
        mode_machine=int(state.mode_machine),
        tick=int(state.tick),
        positions=tuple(float(motor.q) for motor in state.motor_state),
        modes=tuple(int(motor.mode) for motor in state.motor_state),
        mode_pr=int(state.mode_pr),
        reserve=tuple(int(value) for value in state.reserve),
    )
    return frame.with_crc()


def lowcmd_from_frame(frame: LowCmdFrame) -> HGLowCmd:
    """Build a wire-signed HGLowCmd from a neutral LowCmdFrame."""

    _require_slots(frame.motor_cmd, TOTAL_MOTOR_SLOTS)
    cmd = HGLowCmd(
        mode_pr=int(frame.mode_pr),
        mode_machine=int(frame.mode_machine),
        motor_cmd=[
            HGMotorCmd(
                mode=int(command.mode),
                q=float(command.q),
                dq=float(command.dq),
                tau=float(command.tau),
                kp=float(command.kp),
                kd=float(command.kd),
                reserve=int(command.reserve),
            )
            for command in frame.motor_cmd
        ],
        reserve=[int(value) for value in frame.reserve],
    )
    _, cmd.crc = lowcmd_to_wire(cmd)
    return cmd


def convert_hg_lowcmd(cmd: Any) -> LowCmdFrame:
    """Convert one HG LowCmd_ into the neutral LowCmdFrame (CRC-validated)."""

    if hg_lowcmd_crc(cmd) != cmd.crc:
        raise BridgeCRCError(
            f"{CMD_TOPIC} frame failed its HG wire CRC check; refusing to convert"
        )
    frame = LowCmdFrame(
        mode_pr=int(cmd.mode_pr),
        mode_machine=int(cmd.mode_machine),
        motor_cmd=tuple(
            MotorCommand(
                mode=int(motor.mode),
                q=float(motor.q),
                dq=float(motor.dq),
                tau=float(motor.tau),
                kp=float(motor.kp),
                kd=float(motor.kd),
                reserve=int(motor.reserve),
            )
            for motor in cmd.motor_cmd
        ),
        reserve=tuple(int(value) for value in cmd.reserve),
    )
    return frame.with_crc()


# ---------------------------------------------------------------------------
# Live cyclonedds type construction (unitree_hg IDL mirror)
# ---------------------------------------------------------------------------

_CYCLONE_TYPES: dict[str, type] | None = None


def _cyclone_types() -> dict[str, type] | None:
    """Build cyclonedds IdlStruct types mirroring the unitree_hg IDL.

    Returns None when cyclonedds is not importable (repository tests).  The
    generated types carry the exact ``unitree_hg.msg.dds_`` type names and
    sequential autoid annotations so they interoperate with the robot's
    native DDS participants.
    """

    global _CYCLONE_TYPES
    if _CYCLONE_TYPES is not None:
        return _CYCLONE_TYPES
    if not _HAVE_CYCLONEDDS:
        return None

    @dataclass
    @annotate.final
    @annotate.autoid("sequential")
    class IMUState(  # noqa: N801
        idl.IdlStruct, typename="unitree_hg.msg.dds_.IMUState_"
    ):
        quaternion: types.array[types.float32, 4]
        gyroscope: types.array[types.float32, 3]
        accelerometer: types.array[types.float32, 3]
        rpy: types.array[types.float32, 3]
        temperature: types.int16

    @dataclass
    @annotate.final
    @annotate.autoid("sequential")
    class MotorState(  # noqa: N801
        idl.IdlStruct, typename="unitree_hg.msg.dds_.MotorState_"
    ):
        mode: types.uint8
        q: types.float32
        dq: types.float32
        ddq: types.float32
        tau_est: types.float32
        temperature: types.array[types.int16, 2]
        vol: types.float32
        sensor: types.array[types.uint32, 2]
        motorstate: types.uint32
        reserve: types.array[types.uint32, 4]

    @dataclass
    @annotate.final
    @annotate.autoid("sequential")
    class LowState(  # noqa: N801
        idl.IdlStruct, typename="unitree_hg.msg.dds_.LowState_"
    ):
        version: types.array[types.uint32, 2]
        mode_pr: types.uint8
        mode_machine: types.uint8
        tick: types.uint32
        imu_state: IMUState
        motor_state: types.array[MotorState, 35]
        wireless_remote: types.array[types.uint8, 40]
        reserve: types.array[types.uint32, 4]
        crc: types.uint32

    @dataclass
    @annotate.final
    @annotate.autoid("sequential")
    class MotorCmd(  # noqa: N801
        idl.IdlStruct, typename="unitree_hg.msg.dds_.MotorCmd_"
    ):
        mode: types.uint8
        q: types.float32
        dq: types.float32
        tau: types.float32
        kp: types.float32
        kd: types.float32
        reserve: types.uint32

    @dataclass
    @annotate.final
    @annotate.autoid("sequential")
    class LowCmd(  # noqa: N801
        idl.IdlStruct, typename="unitree_hg.msg.dds_.LowCmd_"
    ):
        mode_pr: types.uint8
        mode_machine: types.uint8
        motor_cmd: types.array[MotorCmd, 35]
        reserve: types.array[types.uint32, 4]
        crc: types.uint32

    _CYCLONE_TYPES = {
        "IMUState": IMUState,
        "MotorState": MotorState,
        "LowState": LowState,
        "MotorCmd": MotorCmd,
        "LowCmd": LowCmd,
    }
    return _CYCLONE_TYPES


def _resolve_msg_type(name: str) -> type:
    """Live cyclonedds type when available, plain record otherwise."""

    types = _cyclone_types()
    if types is not None:
        return types[name]
    return {"LowState": HGLowState, "LowCmd": HGLowCmd}[name]


def _import_cyclone() -> Any:
    try:
        import cyclonedds  # type: ignore[import-untyped]
    except ImportError as error:
        raise BridgeUnavailableError(
            "cyclonedds is not importable in this environment; install the "
            "DDS runtime or inject cyclone= for tests"
        ) from error
    return cyclonedds


def _dds(cyclone: Any, name: str) -> Any:
    """Resolve a DDS class on a cyclonedds module or an injected fake.

    The real package exposes its classes on submodules
    (``cyclonedds.domain.DomainParticipant``); injected fakes and the
    re-exported top-level names both resolve directly.
    """

    direct = getattr(cyclone, name, None)
    if direct is not None:
        return direct
    for submodule in ("domain", "topic", "sub", "pub", "core"):
        nested = getattr(getattr(cyclone, submodule, None), name, None)
        if nested is not None:
            return nested
    raise BridgeUnavailableError(f"cyclonedds object {name!r} not found")


def _to_wire_lowcmd(cmd: HGLowCmd) -> Any:
    """Convert a plain HGLowCmd into the live cyclonedds LowCmd_ type."""

    wire_type = _cyclone_types()
    if wire_type is None:
        return cmd  # plain record path (tests / DDS-less environments)
    motor_type = wire_type["MotorCmd"]
    return wire_type["LowCmd"](
        mode_pr=cmd.mode_pr,
        mode_machine=cmd.mode_machine,
        motor_cmd=[
            motor_type(
                mode=m.mode, q=m.q, dq=m.dq, tau=m.tau, kp=m.kp, kd=m.kd,
                reserve=m.reserve,
            )
            for m in cmd.motor_cmd
        ],
        reserve=list(cmd.reserve),
        crc=cmd.crc,
    )


# ---------------------------------------------------------------------------
# DDS endpoints
# ---------------------------------------------------------------------------


class CycloneLowStateSource:
    """LowStateSource that subscribes to the real rt/lowstate topic."""

    def __init__(
        self,
        *,
        cyclone: Any = None,
        domain_id: int = DEFAULT_DOMAIN_ID,
        topic: str = STATE_TOPIC,
        monotonic: Any = time.monotonic,
        msg_type: type | None = None,
    ) -> None:
        if cyclone is None:
            cyclone = _import_cyclone()
        self._monotonic = monotonic
        participant = _dds(cyclone, "DomainParticipant")(domain_id)
        self._reader = _dds(cyclone, "DataReader")(
            _dds(cyclone, "Subscriber")(participant),
            _dds(cyclone, "Topic")(participant, topic, msg_type or _resolve_msg_type("LowState")),
        )

    def read(self) -> tuple[LowStateFrame, float] | None:
        """Newest CRC-valid frame plus its monotonic receive time, or None."""

        latest: tuple[int, LowStateFrame, float] | None = None
        for sample in self._reader.take():
            try:
                frame = convert_hg_lowstate(sample, received_at=0.0)
            except BridgeCRCError:
                continue  # corrupt wire frame: never reaches the adapter
            if latest is None or frame.tick > latest[0]:
                latest = (frame.tick, frame, self._monotonic())
        if latest is None:
            return None
        return latest[1], latest[2]


class CycloneLowCmdSink:
    """Executor command sink that publishes real rt/lowcmd frames.

    Implements the sink protocol (``write(sample)``): every streamed
    :class:`StreamSample` frame is converted to a wire-signed HGLowCmd and
    published at the executor's cadence.
    """

    def __init__(
        self,
        *,
        cyclone: Any = None,
        domain_id: int = DEFAULT_DOMAIN_ID,
        topic: str = CMD_TOPIC,
        msg_type: type | None = None,
    ) -> None:
        if cyclone is None:
            cyclone = _import_cyclone()
        participant = _dds(cyclone, "DomainParticipant")(domain_id)
        self._writer = _dds(cyclone, "DataWriter")(
            _dds(cyclone, "Publisher")(participant),
            _dds(cyclone, "Topic")(participant, topic, msg_type or _resolve_msg_type("LowCmd")),
        )

    def write(self, sample: Any) -> None:
        frame = getattr(sample, "frame", None)
        if frame is None:
            return  # non-frame samples (Dex1 commands) are not lowcmd traffic
        cmd = lowcmd_from_frame(frame)
        self._writer.write(_to_wire_lowcmd(cmd))


__all__ = [
    "BridgeCRCError",
    "BridgeError",
    "BridgeUnavailableError",
    "CMD_TOPIC",
    "CycloneLowCmdSink",
    "CycloneLowStateSource",
    "DEFAULT_DOMAIN_ID",
    "HGIMUState",
    "HGLowCmd",
    "HGLowState",
    "HGMotorCmd",
    "HGMotorState",
    "STATE_TOPIC",
    "convert_hg_lowcmd",
    "convert_hg_lowstate",
    "hg_lowcmd_crc",
    "hg_lowstate_crc",
    "hg_lowstate_pack_size",
    "lowcmd_from_frame",
    "lowcmd_to_wire",
]
