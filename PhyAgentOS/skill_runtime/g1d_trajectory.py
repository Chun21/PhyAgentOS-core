"""Analytically bounded joint-space deceleration for an admitted trajectory."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass


def _value(coefficients: Sequence[float], u: float) -> float:
    value = 0.0
    for c in reversed(coefficients):
        value = value * u + c
    return value


@dataclass(frozen=True)
class StopTrajectory:
    """Synchronized C2 transition from commanded q/dq/ddq to rest."""

    duration_s: float
    coefficients: tuple[tuple[float, ...], ...]

    def sample(self, elapsed_s: float) -> tuple[tuple[float, ...], tuple[float, ...]]:
        u = min(1.0, max(0.0, elapsed_s / self.duration_s))
        q, dq = [], []
        for c in self.coefficients:
            q.append(_value(c, u))
            dq.append(
                _value([i * c[i] for i in range(1, 5)], u) / self.duration_s if u < 1 else 0.0
            )
        return tuple(q), tuple(dq)


def _within(
    c: Sequence[float],
    bounds: tuple[float, float],
    numerator: float = 0.0,
    denominator: float = 0.0,
) -> bool:
    points = [0.0, 1.0]
    if denominator != 0 and 0 < numerator / denominator < 1:
        points.append(numerator / denominator)
    return all(bounds[0] - 1e-9 <= _value(c, u) <= bounds[1] + 1e-9 for u in points)


def plan_stop(
    q: Sequence[float],
    dq: Sequence[float],
    ddq: Sequence[float],
    *,
    limits: Sequence[tuple[float, float]],
    velocity: float,
    acceleration: float,
    jerk: float,
    budget_s: float,
) -> StopTrajectory:
    """Find a bounded stop or reject when no verified stop fits the budget.

    Integrate cubic velocity with incoming velocity/acceleration and zero
    terminal velocity/acceleration. Its factorization (1-u)^2*(v+(2v+a)*u)
    gives every interior extremum of position and each derivative exactly.
    """
    for step in range(80):
        duration = 0.05 + (budget_s - 0.05) * step / 79
        coefficients = []
        for position, speed, accel, bounds in zip(q, dq, ddq, limits, strict=True):
            v, a = speed * duration, accel * duration**2
            c = (position, v, a / 2, -v - 2 * a / 3, v / 2 + a / 4)
            d1 = tuple(i * c[i] / duration for i in range(1, 5))
            d2 = tuple(i * d1[i] / duration for i in range(1, 4))
            d3 = tuple(i * d2[i] / duration for i in range(1, 3))
            if not (
                _within(c, bounds, -v, 2 * v + a)
                and _within(d1, (-velocity, velocity), a, 6 * v + 3 * a)
                and _within(d2, (-acceleration, acceleration), 6 * v + 4 * a, 12 * v + 6 * a)
                and _within(d3, (-jerk, jerk))
            ):
                break
            coefficients.append(c)
        else:
            return StopTrajectory(duration, tuple(coefficients))
    raise ValueError("no bounded deceleration fits the stop margin and joint limits")


def quintic_duration(
    start: Sequence[float],
    target: Sequence[float],
    *,
    minimum_duration_s: float,
    max_velocity: float,
    max_acceleration: float,
    max_jerk: float,
) -> float:
    """Common planning/execution duration from analytic quintic peak factors."""
    delta = max((abs(b - a) for a, b in zip(start, target, strict=True)), default=0.0)
    return max(
        minimum_duration_s,
        1.875 * delta / max_velocity,
        math.sqrt(10 * math.sqrt(3) / 3 * delta / max_acceleration),
        (60 * delta / max_jerk) ** (1 / 3),
    )
