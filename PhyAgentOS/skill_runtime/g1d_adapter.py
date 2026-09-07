"""Robot-adapter seam for the visual-free G1_D manipulation Skill.

The adapter deliberately works with small, transport-neutral frame objects.  A
real DDS/SDK bridge can translate HG messages into :class:`LowStateFrame` and
publish :class:`LowCmdFrame`; the PAOS side never needs to import either SDK.
"""

from __future__ import annotations

import struct
import zlib
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
from typing import Any, Protocol

TOTAL_MOTOR_SLOTS = 35
ARM_SLOTS = tuple(range(15, 29))
LEFT_ARM_SLOTS = tuple(range(15, 22))
RIGHT_ARM_SLOTS = tuple(range(22, 29))


class AdapterError(RuntimeError):
    """Base class for explicit robot-adapter readiness and frame errors."""


class CRCError(AdapterError):
    """A state or command frame failed its CRC check."""


class StateUnavailableError(AdapterError):
    """No valid state has been received yet."""


class StaleStateError(AdapterError):
    """The latest valid state is older than the configured freshness window."""


class ModeLossError(AdapterError):
    """The robot no longer reports an approved mode."""


class SafetyFaultError(AdapterError):
    """The robot safety condition is not healthy."""


def _validate_slots(values: Sequence[float | int], name: str) -> tuple[float, ...]:
    if len(values) != TOTAL_MOTOR_SLOTS:
        raise ValueError(f"{name} must contain {TOTAL_MOTOR_SLOTS} entries")
    return tuple(float(value) for value in values)


def _validate_modes(values: Sequence[int]) -> tuple[int, ...]:
    if len(values) != TOTAL_MOTOR_SLOTS:
        raise ValueError(f"mode must contain {TOTAL_MOTOR_SLOTS} entries")
    return tuple(int(value) for value in values)


@dataclass(frozen=True)
class LowStateFrame:
    """Transport-neutral subset of an HG ``LowState_`` frame."""

    mode_machine: int
    tick: int
    positions: tuple[float, ...]
    modes: tuple[int, ...]
    safety_ok: bool = True
    mode_pr: int = 0
    reserve: tuple[int, int, int, int] = (0, 0, 0, 0)
    crc: int = 0

    def payload(self) -> bytes:
        values: list[int | float] = [
            self.mode_machine,
            self.tick,
            int(self.safety_ok),
            self.mode_pr,
        ]
        values.extend(self.reserve)
        values.extend(self.modes)
        for position in self.positions:
            values.append(position)
        return struct.pack("<2I2B4I35B35f", *values)

    def calculated_crc(self) -> int:
        return zlib.crc32(self.payload()) & 0xFFFFFFFF

    def with_crc(self) -> LowStateFrame:
        return replace(self, crc=self.calculated_crc())

    def to_bytes(self) -> bytes:
        return self.payload() + struct.pack("<I", self.crc)

    @classmethod
    def from_bytes(cls, raw: bytes) -> LowStateFrame:
        payload_size = struct.calcsize("<2I2B4I35B35f")
        if len(raw) != payload_size + 4:
            raise ValueError(f"HG lowstate frame must contain {payload_size + 4} bytes")
        values = struct.unpack("<2I2B4I35B35fI", raw)
        return cls(
            mode_machine=values[0],
            tick=values[1],
            safety_ok=bool(values[2]),
            mode_pr=values[3],
            reserve=tuple(values[4:8]),  # type: ignore[arg-type]
            modes=tuple(values[8:43]),
            positions=tuple(values[43:78]),
            crc=values[78],
        )

    @classmethod
    def from_frame(cls, frame: LowStateFrame, **changes: object) -> LowStateFrame:
        values = {
            "mode_machine": frame.mode_machine,
            "tick": frame.tick,
            "positions": frame.positions,
            "modes": frame.modes,
            "safety_ok": frame.safety_ok,
            "mode_pr": frame.mode_pr,
            "reserve": frame.reserve,
            "crc": frame.crc,
        }
        values.update(changes)
        return cls(**values)  # type: ignore[arg-type]


@dataclass(frozen=True)
class MotorCommand:
    """Publicly inspectable command slot, without vendor transport details."""

    mode: int
    q: float
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    reserve: int = 0


@dataclass(frozen=True)
class LowCmdFrame:
    """Complete 35-slot HG command frame emitted by the adapter."""

    mode_machine: int
    motor_cmd: tuple[MotorCommand, ...]
    mode_pr: int = 0
    reserve: tuple[int, int, int, int] = (0, 0, 0, 0)
    crc: int = 0

    def payload(self) -> bytes:
        values: list[int | float] = [self.mode_pr, self.mode_machine]
        for command in self.motor_cmd:
            values.extend(
                [command.mode, command.q, command.dq, command.tau, command.kp, command.kd, command.reserve]
            )
        values.extend(self.reserve)
        return struct.pack("<2B" + "B3x5fI" * TOTAL_MOTOR_SLOTS + "4I", *values)

    def calculated_crc(self) -> int:
        return zlib.crc32(self.payload()) & 0xFFFFFFFF

    def with_crc(self) -> LowCmdFrame:
        return replace(self, crc=self.calculated_crc())


@dataclass(frozen=True)
class MotorSnapshot:
    slot: int
    q: float
    mode: int


@dataclass(frozen=True)
class Dex1Snapshot:
    available: bool = False


@dataclass(frozen=True)
class AdapterState:
    mode_machine: int
    frame: int
    state_age_ms: float
    safety_gate: str
    left_arm: tuple[MotorSnapshot, ...]
    right_arm: tuple[MotorSnapshot, ...]
    dex1: Dex1Snapshot
    active_operation: str | None


class LowStateSource(Protocol):
    def read(self) -> tuple[LowStateFrame, float] | None:
        """Return the newest frame and its monotonic receive timestamp."""


class FakeLowStateSource:
    """Deterministic state source used by adapter tests and local development."""

    def __init__(self) -> None:
        self._item: tuple[LowStateFrame, float] | None = None

    def publish(self, frame: LowStateFrame, *, received_at: float) -> None:
        self._item = (frame, received_at)

    def read(self) -> tuple[LowStateFrame, float] | None:
        return self._item


def decode_lowstate(raw: LowStateFrame | bytes | Mapping[str, Any] | Any) -> LowStateFrame:
    """Decode a fake wire frame or SDK-shaped object into the public frame type."""

    if isinstance(raw, LowStateFrame):
        return raw
    if isinstance(raw, (bytes, bytearray, memoryview)):
        return LowStateFrame.from_bytes(bytes(raw))
    if isinstance(raw, Mapping):
        get = raw.get
    else:
        def get(name: str, default: Any = None) -> Any:
            return getattr(raw, name, default)
    motor_states = get("motor_state", get("motor_states"))
    positions = get("positions")
    modes = get("modes", get("mode"))
    if motor_states is not None:
        if isinstance(motor_states, (str, bytes)):
            raise ValueError("motor_state must be a sequence")
        positions = [
            item.get("q") if isinstance(item, Mapping) else item.q for item in motor_states
        ]
        modes = [
            item.get("mode", 0) if isinstance(item, Mapping) else item.mode
            for item in motor_states
        ]
    if positions is None:
        raise ValueError("lowstate frame has no motor positions")
    frame = LowStateFrame(
        mode_machine=int(get("mode_machine", 0)),
        tick=int(get("tick", get("frame", 0))),
        positions=_validate_slots(positions, "positions"),
        modes=_validate_modes(modes or [0] * TOTAL_MOTOR_SLOTS),
        safety_ok=bool(get("safety_ok", True)),
        mode_pr=int(get("mode_pr", 0)),
        reserve=tuple(get("reserve", (0, 0, 0, 0))),  # type: ignore[arg-type]
        crc=int(get("crc", 0)),
    )
    return frame


def make_lowstate_frame(
    *,
    mode_machine: int,
    tick: int,
    positions: Sequence[float | int],
    mode: Sequence[int] | None = None,
    safety_ok: bool = True,
) -> LowStateFrame:
    frame = LowStateFrame(
        mode_machine=int(mode_machine),
        tick=int(tick),
        positions=_validate_slots(positions, "positions"),
        modes=_validate_modes(mode or [0] * TOTAL_MOTOR_SLOTS),
        safety_ok=bool(safety_ok),
    )
    return frame.with_crc()


class G1DAdapter:
    """Validate HG state and produce complete current-position hold frames."""

    def __init__(
        self,
        *,
        clock: Callable[[], float],
        max_state_age_s: float = 0.1,
        approved_mode_machine: int | None = None,
    ) -> None:
        if max_state_age_s <= 0:
            raise ValueError("max_state_age_s must be positive")
        self._clock = clock
        self._max_state_age_s = max_state_age_s
        self._approved_mode_machine = approved_mode_machine
        self._latest: LowStateFrame | None = None
        self._received_at: float | None = None
        self._active_operation: str | None = None

    def ingest(
        self,
        frame: LowStateFrame | bytes | Mapping[str, Any] | Any,
        *,
        received_at: float | None = None,
    ) -> None:
        frame = decode_lowstate(frame)
        if frame.calculated_crc() != frame.crc:
            raise CRCError("rt/lowstate CRC mismatch")
        if received_at is None:
            received_at = self._clock()
        self._latest = frame
        self._received_at = float(received_at)

    def poll(self, source: LowStateSource) -> AdapterState:
        item = source.read()
        if item is None:
            raise StateUnavailableError("no rt/lowstate frame available")
        frame, received_at = item
        self.ingest(frame, received_at=received_at)
        return self.query_state()

    def _require_frame(self) -> LowStateFrame:
        if self._latest is None or self._received_at is None:
            raise StateUnavailableError("no CRC-valid rt/lowstate frame has been received")
        age = self._clock() - self._received_at
        if age > self._max_state_age_s:
            raise StaleStateError(f"rt/lowstate is stale ({age * 1000:.1f} ms)")
        return self._latest

    def require_ready(self) -> AdapterState:
        frame = self._require_frame()
        if self._approved_mode_machine is None:
            raise ModeLossError("approved mode_machine is not configured")
        if frame.mode_machine == 0 or frame.mode_machine != self._approved_mode_machine:
            raise ModeLossError("approved mode_machine is not present")
        if not frame.safety_ok:
            raise SafetyFaultError("Safety gate reports a fault")
        return self.query_state()

    def query_state(self) -> AdapterState:
        frame = self._require_frame()
        received_at = self._received_at
        assert received_at is not None
        age_ms = max(0.0, (self._clock() - received_at) * 1000.0)
        if not frame.safety_ok:
            safety_gate = "fault"
        elif frame.mode_machine == 0:
            safety_gate = "mode_lost"
        else:
            safety_gate = "ready"
        snapshots = tuple(
            MotorSnapshot(slot=index, q=frame.positions[index], mode=frame.modes[index])
            for index in ARM_SLOTS
        )
        return AdapterState(
            mode_machine=frame.mode_machine,
            frame=frame.tick,
            state_age_ms=age_ms,
            safety_gate=safety_gate,
            left_arm=tuple(snapshots[:7]),
            right_arm=tuple(snapshots[7:]),
            dex1=Dex1Snapshot(),
            active_operation=self._active_operation,
        )

    def hold_frame(self) -> LowCmdFrame:
        self.require_ready()
        frame = self._latest
        assert frame is not None
        commands = tuple(
            MotorCommand(mode=frame.modes[index], q=frame.positions[index])
            for index in range(TOTAL_MOTOR_SLOTS)
        )
        return LowCmdFrame(
            mode_machine=frame.mode_machine,
            mode_pr=frame.mode_pr,
            motor_cmd=commands,
            reserve=frame.reserve,
        ).with_crc()

    def set_active_operation(self, operation_id: str | None) -> None:
        self._active_operation = operation_id

__all__ = [
    "AdapterError",
    "AdapterState",
    "ARM_SLOTS",
    "CRCError",
    "Dex1Snapshot",
    "FakeLowStateSource",
    "G1DAdapter",
    "LEFT_ARM_SLOTS",
    "LowCmdFrame",
    "LowStateFrame",
    "ModeLossError",
    "MotorCommand",
    "MotorSnapshot",
    "RIGHT_ARM_SLOTS",
    "SafetyFaultError",
    "StateUnavailableError",
    "StaleStateError",
    "TOTAL_MOTOR_SLOTS",
    "make_lowstate_frame",
    "decode_lowstate",
]
