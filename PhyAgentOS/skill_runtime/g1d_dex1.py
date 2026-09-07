"""Optional external Dex1-1 opening and state integration.

The Dex1-1 serial-to-DDS service is an *external robot service*: it is installed
and supervised outside this Skill Bundle, and normal Tool calls never install,
calibrate, restart, or stop it.  This module only reads its
``rt/dex1/{left,right}/state`` stream, validates normalized opening commands
for ``rt/dex1/{left,right}/cmd``, and gates *requested* execution on state
freshness.  A missing opening is non-blocking: Dex1 is optional.
"""

from __future__ import annotations

import enum
import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any, Protocol

DEX1_SIDES = ("left", "right")
DEFAULT_DEX1_MAX_STATE_AGE_S = 0.1


class Dex1Error(RuntimeError):
    """Base class for explicit Dex1 integration errors."""


class InvalidOpeningError(Dex1Error):
    """An opening value is non-finite or outside [0, 1]."""


class Dex1NotReadyError(Dex1Error):
    """A requested Dex1 side is absent, stale, or timed out."""


def validate_opening(value: Any) -> float:
    """Opening values must be finite and within the normalized [0, 1] range."""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InvalidOpeningError(f"Dex1 opening must be a number, got {value!r}")
    opening = float(value)
    if not (math.isfinite(opening) and 0.0 <= opening <= 1.0):
        raise InvalidOpeningError(f"Dex1 opening must be finite and within [0, 1], got {value!r}")
    return opening


class Dex1Status(enum.Enum):
    """Readiness of one Dex1 side's external state stream."""

    HEALTHY = "healthy"
    ABSENT = "absent"
    STALE = "stale"


@dataclass(frozen=True)
class Dex1Command:
    """One normalized opening command for the external service."""

    side: str
    opening: float
    topic: str


@dataclass(frozen=True)
class Dex1CommandSample:
    """One Dex1 opening command as emitted during execution."""

    t_s: float
    command: Dex1Command


@dataclass(frozen=True)
class Dex1Readiness:
    """Freshness and timeout indication for one Dex1 side."""

    side: str
    status: Dex1Status
    opening: float | None = None
    age_ms: float | None = None

    @property
    def ok(self) -> bool:
        return self.status is Dex1Status.HEALTHY

    @property
    def timed_out(self) -> bool:
        """A state stream that exists but has exceeded its freshness window."""

        return self.status is Dex1Status.STALE


class Dex1StateSource(Protocol):
    def read(self, side: str) -> tuple[float, float] | None:
        """Return the newest opening and its monotonic receive timestamp."""


class FakeDex1StateSource:
    """Deterministic per-side Dex1 state source for tests and development."""

    def __init__(self) -> None:
        self._items: dict[str, tuple[float, float]] = {}

    def publish(self, opening: float, *, received_at: float, side: str = "left") -> None:
        self._items[side] = (validate_opening(opening), float(received_at))

    def read(self, side: str) -> tuple[float, float] | None:
        return self._items.get(side)


class Dex1Integration:
    """Read external Dex1 state and validate normalized opening commands.

    The integration never touches the service lifecycle: no install, no
    calibration, no restart, no stop.  It only observes state and produces
    command values for the external ``rt/dex1/{side}/cmd`` topics.
    """

    def __init__(
        self,
        *,
        clock: Callable[[], float],
        max_state_age_s: float = DEFAULT_DEX1_MAX_STATE_AGE_S,
        sources: Mapping[str, Dex1StateSource] | None = None,
    ) -> None:
        if max_state_age_s <= 0:
            raise Dex1Error("max_state_age_s must be positive")
        self._clock = clock
        self._max_state_age_s = float(max_state_age_s)
        self._latest: dict[str, tuple[float, float]] = {}
        self._sources: dict[str, Dex1StateSource] = dict(sources or {})
        for side in self._sources:
            _validate_side(side)

    def publish(self, side: str, *, opening: float, received_at: float) -> None:
        """Ingest one external state sample for a side."""

        _validate_side(side)
        self._latest[side] = (validate_opening(opening), float(received_at))

    def readiness(self, side: str) -> Dex1Readiness:
        """Freshness and timeout indication for one side."""

        _validate_side(side)
        source = self._sources.get(side)
        if source is not None:
            item = source.read(side)
            if item is not None:
                self._latest[side] = (validate_opening(item[0]), float(item[1]))
        item = self._latest.get(side)
        if item is None:
            return Dex1Readiness(side=side, status=Dex1Status.ABSENT)
        opening, received_at = item
        age_s = self._clock() - received_at
        if age_s > self._max_state_age_s:
            return Dex1Readiness(
                side=side, status=Dex1Status.STALE, opening=opening, age_ms=age_s * 1000.0
            )
        return Dex1Readiness(
            side=side,
            status=Dex1Status.HEALTHY,
            opening=opening,
            age_ms=max(0.0, age_s * 1000.0),
        )

    def command(self, side: str, opening: float) -> Dex1Command:
        """Validated normalized opening command for the external topic."""

        _validate_side(side)
        return Dex1Command(
            side=side, opening=validate_opening(opening), topic=f"rt/dex1/{side}/cmd"
        )

    def require_ready_for(
        self,
        requested: Mapping[str, float | None],
    ) -> None:
        """Gate execution on the readiness of every *requested* side.

        Sides without a requested opening are ignored: a missing opening is
        non-blocking, and the external service may be entirely absent.
        """

        for side in DEX1_SIDES:
            opening = requested.get(side)
            if opening is None:
                continue
            # The requested value itself must be valid before the gate.
            validate_opening(opening)
            readiness = self.readiness(side)
            if not readiness.ok:
                raise Dex1NotReadyError(
                    f"requested Dex1 {side} state is {readiness.status.value}"
                    + (
                        f" (age {readiness.age_ms:.1f} ms, timed out)"
                        if readiness.timed_out
                        else ""
                    )
                )


def _validate_side(side: str) -> str:
    if side not in DEX1_SIDES:
        raise Dex1Error(f"Dex1 side must be one of {DEX1_SIDES}, got {side!r}")
    return side


__all__ = [
    "DEX1_SIDES",
    "Dex1Command",
    "Dex1CommandSample",
    "Dex1Status",
    "Dex1Error",
    "Dex1Integration",
    "Dex1NotReadyError",
    "Dex1Readiness",
    "Dex1StateSource",
    "FakeDex1StateSource",
    "InvalidOpeningError",
    "validate_opening",
]
