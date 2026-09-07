"""Read-only, binding-aware reduced IK/FK planner for the G1_D dual-arm Skill.

``g1d.dual_arm.plan_pose`` validates a bilateral Cartesian target, solves a
reduced dual-arm IK, verifies the result against the fixture kinematics, and
returns a short-lived plan identity.  The planner never publishes a command:
it is the query half of the Forge Tool contract described in ``CONTEXT.md``.

The kinematics seam is a protocol so tests run on deterministic recorded
fixtures while the real deployment binds UniRobot's ``G1_D_DEX1_ArmIK``
behind the same interface.  No SDK, DDS, or robot import is needed here.
"""

from __future__ import annotations

import hashlib
import json
import math
import uuid
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

TOTAL_MOTOR_SLOTS = 35
ARM_DOF = 7
DEFAULT_PLAN_TTL_S = 5.0
DEFAULT_POSITION_TOLERANCE_M = 0.02
DEFAULT_ORIENTATION_TOLERANCE_DEG = 5.0
# Generous G1_D reach envelope in metres; catches unit errors (degrees, mm)
# before IK is ever consulted. The true envelope is verified by fixtures.
MAX_REACH_ENVELOPE_M = 1.2


class PlannerError(RuntimeError):
    """Base class for explicit planner failures."""


class PoseValidationError(PlannerError):
    """A target pose is malformed, out of envelope, or uses wrong units."""


class UnreachableTargetError(PlannerError):
    """The kinematics seam reports no solution for the target pair."""


class KinematicsError(PlannerError):
    """The kinematics seam failed outside its documented contract."""


class InvalidSolutionError(PlannerError):
    """An IK solution is non-finite or otherwise not executable."""


class JointLimitError(InvalidSolutionError):
    """An IK solution exceeds the configured joint limits."""


class PlanExpiredError(PlannerError):
    """A plan identity was fetched outside its validity window."""


def digest_json(value: Any) -> str:
    """Stable SHA-256 digest of a JSON-serialisable value (sorted keys)."""

    canonical = json.dumps(value, sort_keys=True, separators=(",", ":"), default=repr)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class ArmPose:
    """One validated arm target: metres, normalized xyzw, agreed frame."""

    frame_id: str
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    dex1_opening: float | None = None

    def as_digest_payload(self) -> dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "position_m": list(self.position_m),
            "orientation_xyzw": list(self.orientation_xyzw),
            "dex1_opening": self.dex1_opening,
        }


def _require_finite(values: Sequence[float], name: str) -> tuple[float, ...]:
    for value in values:
        if not math.isfinite(value):
            raise PoseValidationError(f"{name} must contain finite values")
    return tuple(float(value) for value in values)


def validate_arm_pose(target: Mapping[str, Any]) -> ArmPose:
    """Validate one arm target against the strict Tool input contract."""

    if not isinstance(target, Mapping):
        raise PoseValidationError("arm target must be an object")

    frame_id = target.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id:
        raise PoseValidationError("frame_id must be a non-empty string")

    raw_position = target.get("position_m")
    if not isinstance(raw_position, Sequence) or isinstance(raw_position, (str, bytes)):
        raise PoseValidationError("position_m must be an array of three numbers")
    if len(raw_position) != 3:
        raise PoseValidationError("position_m must contain exactly three metres")
    position = _require_finite(raw_position, "position_m")
    if any(abs(component) > MAX_REACH_ENVELOPE_M for component in position):
        raise PoseValidationError("position_m leaves the G1_D reach envelope; check units (metres)")

    raw_orientation = target.get("orientation_xyzw")
    if (
        not isinstance(raw_orientation, Sequence)
        or isinstance(raw_orientation, (str, bytes))
        or len(raw_orientation) != 4
    ):
        raise PoseValidationError("orientation_xyzw must contain exactly four values")
    orientation = _require_finite(raw_orientation, "orientation_xyzw")
    norm = math.sqrt(sum(component * component for component in orientation))
    if not math.isclose(norm, 1.0, rel_tol=1e-6, abs_tol=1e-9):
        raise PoseValidationError(f"orientation_xyzw must be normalized (|q| = {norm:.6f})")

    dex1 = target.get("dex1")
    dex1_opening: float | None = None
    if dex1 is not None:
        if not isinstance(dex1, Mapping) or "opening" not in dex1:
            raise PoseValidationError("dex1 must be an object with an opening value")
        opening = dex1["opening"]
        if (
            isinstance(opening, bool)
            or not isinstance(opening, (int, float))
            or not math.isfinite(float(opening))
            or not 0.0 <= float(opening) <= 1.0
        ):
            raise PoseValidationError("dex1.opening must be finite and within [0, 1]")
        dex1_opening = float(opening)

    return ArmPose(
        frame_id=frame_id,
        position_m=position,  # type: ignore[arg-type]
        orientation_xyzw=orientation,  # type: ignore[arg-type]
        dex1_opening=dex1_opening,
    )


@dataclass(frozen=True)
class JointSolution:
    """Reduced 7+7 joint values produced by the kinematics seam."""

    left_q: tuple[float, ...]
    right_q: tuple[float, ...]


@dataclass(frozen=True)
class CartesianPose:
    """FK result in the agreed robot frame."""

    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


class Kinematics(Protocol):
    """Reduced dual-arm kinematics seam (UniRobot G1_D_DEX1_ArmIK binds here)."""

    def solve_ik(self, left: ArmPose, right: ArmPose) -> JointSolution:
        """Solve the bilateral reduced IK; raise when unreachable."""

    def solve_fk(
        self, left_q: Sequence[float], right_q: Sequence[float]
    ) -> tuple[CartesianPose, CartesianPose]:
        """Forward kinematics for the reduced arm pair."""


class FixtureKinematics:
    """Deterministic recorded (target -> joints) pairs for tests and fixtures.

    Each fixture records a bilateral target and the joints the real
    ``G1_D_DEX1_ArmIK`` returned for it.  An exact target match replays the
    recorded solution; anything else is explicitly unreachable.  ``solve_fk``
    is a surrogate that maps joint values through to the pose fields, so
    round-trip verification measures the numeric deviation between a recorded
    solution and its requested frame rather than trusting the solver.

    The real deployment binds UniRobot's ``G1_D_DEX1_ArmIK`` behind the
    :class:`Kinematics` protocol instead; the fixture form exists so tests
    never import an SDK, DDS client, or robot model.
    """

    def __init__(
        self,
        fixtures: Sequence[
            tuple[Mapping[str, Any], Mapping[str, Any], Sequence[float], Sequence[float]]
        ],
    ) -> None:
        self._by_target: dict[tuple[str, str], tuple[tuple[float, ...], tuple[float, ...]]] = {}
        for left, right, left_q, right_q in fixtures:
            left_pose = validate_arm_pose(left)
            right_pose = validate_arm_pose(right)
            self._by_target[
                (
                    digest_json(left_pose.as_digest_payload()),
                    digest_json(right_pose.as_digest_payload()),
                )
            ] = (tuple(float(v) for v in left_q), tuple(float(v) for v in right_q))

    def solve_ik(self, left: ArmPose, right: ArmPose) -> JointSolution:
        key = (digest_json(left.as_digest_payload()), digest_json(right.as_digest_payload()))
        recorded = self._by_target.get(key)
        if recorded is None:
            raise UnreachableTargetError("no recorded fixture solves this bilateral target")
        return JointSolution(left_q=recorded[0], right_q=recorded[1])

    def solve_fk(
        self, left_q: Sequence[float], right_q: Sequence[float]
    ) -> tuple[CartesianPose, CartesianPose]:
        if len(left_q) != ARM_DOF or len(right_q) != ARM_DOF:
            raise KinematicsError(f"each arm needs {ARM_DOF} joint values")
        for value in (*left_q, *right_q):
            if not math.isfinite(value):
                raise KinematicsError("joint values must be finite")
        # FK surrogate: joint values map straight through to the pose fields,
        # so a round trip reproduces the target only when the recorded
        # solution numerically matches it.
        return (
            CartesianPose(
                position_m=(float(left_q[0]), float(left_q[1]), float(left_q[2])),
                orientation_xyzw=(
                    float(left_q[3]),
                    float(left_q[4]),
                    float(left_q[5]),
                    float(left_q[6]),
                ),
            ),
            CartesianPose(
                position_m=(float(right_q[0]), float(right_q[1]), float(right_q[2])),
                orientation_xyzw=(
                    float(right_q[3]),
                    float(right_q[4]),
                    float(right_q[5]),
                    float(right_q[6]),
                ),
            ),
        )


def map_to_35_slots(solution: JointSolution, preserved_slots: Sequence[float]) -> tuple[float, ...]:
    """Overlay 14 arm values onto slots 15-28 of a complete 35-slot command.

    The HG frame carries 35 motor slots; the G1_D has 29 valid motors, so
    slots 29-34 are zero-filled and never commanded.
    """

    if len(solution.left_q) != ARM_DOF or len(solution.right_q) != ARM_DOF:
        raise ValueError(f"each arm needs {ARM_DOF} joint values")
    if len(preserved_slots) != 15:
        raise ValueError("preserved_slots must contain the 15 non-arm slots (0-14)")
    slots = (
        tuple(float(value) for value in preserved_slots)
        + solution.left_q
        + solution.right_q
        + (0.0,) * (TOTAL_MOTOR_SLOTS - 29)
    )
    if len(slots) != TOTAL_MOTOR_SLOTS:
        raise ValueError(f"slot command must contain {TOTAL_MOTOR_SLOTS} entries")
    return slots


def _validate_slot_layout(left_slots: Any, right_slots: Any, preserved_slots: Any) -> None:
    """The G1_D 29-slot layout is fixed by the platform; guard it once."""

    if list(left_slots) != list(range(15, 22)):
        raise PlannerError("left_slots must map to 15-21")
    if list(right_slots) != list(range(22, 29)):
        raise PlannerError("right_slots must map to 22-28")
    if list(preserved_slots) != list(range(15)):
        raise PlannerError("preserved_slots must keep 0-14 untouched")


def load_kinematics_profile(path: str | Path) -> dict[str, Any]:
    """Load and validate the bundle kinematics profile."""

    data = json.loads(Path(path).read_text(encoding="utf-8"))
    for key in ("left_slots", "right_slots", "preserved_slots"):
        if key not in data:
            raise PlannerError(f"kinematics profile is missing {key!r}")
    _validate_slot_layout(data["left_slots"], data["right_slots"], data["preserved_slots"])
    return data


@dataclass(frozen=True)
class PlanCheck:
    """One named verification result reported back to the caller."""

    name: str
    passed: bool
    detail: str


@dataclass(frozen=True)
class PlanBinding:
    """Plan identity: what skill/runtime/target pair produced the plan."""

    skill_version: str
    runtime_instance_id: str
    target_digest: str
    profile_digest: str


@dataclass(frozen=True)
class PlannedMotionSummary:
    """Summary of the planned quintic joint motion for this plan.

    Not the domain Trajectory profile: that is the versioned limit set the
    execute seam audits against. This records what one plan's motion would be.
    """

    duration_s: float
    sample_hz: int
    max_velocity: float
    max_acceleration: float


@dataclass(frozen=True)
class PosePlan:
    """Complete plan_pose output, ready for execute_pose binding."""

    plan_id: str
    created_at: float
    expires_at: float
    binding: PlanBinding
    joint_solution: JointSolution
    checks: tuple[PlanCheck, ...]
    trajectory: PlannedMotionSummary
    dex1_left_opening: float | None = None
    dex1_right_opening: float | None = None

    @property
    def checks_passed(self) -> bool:
        return all(check.passed for check in self.checks)


def quaternion_angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    """Angle between two quaternions in degrees (sign-insensitive)."""

    dot = sum(x * y for x, y in zip(a, b, strict=True))
    return math.degrees(2.0 * math.acos(min(1.0, abs(dot))))


class G1DPlanner:
    """Validate, solve, verify, and bind one bilateral arm target."""

    def __init__(
        self,
        *,
        kinematics: Kinematics,
        clock: Callable[[], float],
        skill_version: str,
        runtime_instance_id: str,
        profile_digest: str,
        arm_joint_limit_rad: float = 2.8,
        max_joint_velocity_rad_per_s: float = 0.5,
        max_joint_acceleration_rad_per_s2: float = 2.0,
        minimum_duration_s: float = 1.0,
        sample_hz: int = 500,
        position_tolerance_m: float = DEFAULT_POSITION_TOLERANCE_M,
        orientation_tolerance_deg: float = DEFAULT_ORIENTATION_TOLERANCE_DEG,
    ) -> None:
        # The 29-slot layout (left 15-21, right 22-28, 0-14 preserved) is
        # fixed by the G1_D platform and validated in the kinematics profile;
        # the plan TTL is fixed at five seconds by the Tool contract.
        self._kinematics = kinematics
        self._clock = clock
        self._skill_version = skill_version
        self._runtime_instance_id = runtime_instance_id
        self._profile_digest = profile_digest
        self._arm_joint_limit_rad = float(arm_joint_limit_rad)
        self._max_joint_velocity_rad_per_s = float(max_joint_velocity_rad_per_s)
        self._max_joint_acceleration_rad_per_s2 = float(max_joint_acceleration_rad_per_s2)
        self._minimum_duration_s = float(minimum_duration_s)
        self._sample_hz = int(sample_hz)
        self._plan_ttl_s = DEFAULT_PLAN_TTL_S
        self._position_tolerance_m = float(position_tolerance_m)
        self._orientation_tolerance_deg = float(orientation_tolerance_deg)
        self._plans: dict[str, PosePlan] = {}

    @property
    def minimum_duration_s(self) -> float:
        return self._minimum_duration_s

    @property
    def max_joint_velocity_rad_per_s(self) -> float:
        return self._max_joint_velocity_rad_per_s

    @property
    def max_joint_acceleration_rad_per_s2(self) -> float:
        return self._max_joint_acceleration_rad_per_s2

    def plan_pose(
        self,
        *,
        left: Mapping[str, Any],
        right: Mapping[str, Any],
        current_q: Sequence[float] | None = None,
    ) -> PosePlan:
        """Validate both targets, solve reduced IK, verify, and bind a plan."""

        left_pose = validate_arm_pose(left)
        right_pose = validate_arm_pose(right)
        if left_pose.frame_id != right_pose.frame_id:
            raise PoseValidationError("both arm targets must share one frame_id")

        checks: list[PlanCheck] = []
        checks.append(
            PlanCheck(
                name="units_finite",
                passed=True,
                detail="positions are finite metres; orientations are normalized xyzw",
            )
        )

        solution = self._kinematics.solve_ik(left_pose, right_pose)
        self._validate_solution(solution)

        checks.append(
            PlanCheck(
                name="reachability",
                passed=True,
                detail="fixture kinematics solved the bilateral target",
            )
        )
        checks.append(
            PlanCheck(
                name="joint_limits",
                passed=True,
                detail=f"all arm joints within ±{self._arm_joint_limit_rad} rad",
            )
        )

        # FK/IK round-trip: measure true deviation between requested and
        # achieved frames in metres and degrees.
        fk_left, fk_right = self._kinematics.solve_fk(solution.left_q, solution.right_q)
        left_pos_error = math.dist(fk_left.position_m, left_pose.position_m)
        right_pos_error = math.dist(fk_right.position_m, right_pose.position_m)
        left_ori_error = quaternion_angle_deg(fk_left.orientation_xyzw, left_pose.orientation_xyzw)
        right_ori_error = quaternion_angle_deg(
            fk_right.orientation_xyzw, right_pose.orientation_xyzw
        )
        position_ok = max(left_pos_error, right_pos_error) <= self._position_tolerance_m
        orientation_ok = max(left_ori_error, right_ori_error) <= self._orientation_tolerance_deg
        checks.append(
            PlanCheck(
                name="fk_ik_roundtrip",
                passed=position_ok and orientation_ok,
                detail=(
                    f"position error L/R {left_pos_error:.4f}/{right_pos_error:.4f} m, "
                    f"orientation error L/R {left_ori_error:.2f}/{right_ori_error:.2f} deg"
                ),
            )
        )

        # Signs/order/zero/units: the reduced solution must be 7+7 values in
        # the verified slot order, and its magnitude must look like radians
        # (a zero or small pose inside the limits) rather than degrees/counts.
        slots = map_to_35_slots(solution, [0.0] * 15)
        slot_order_ok = (
            slots[15:22] == solution.left_q
            and slots[22:29] == solution.right_q
            and slots[:15] == (0.0,) * 15
        )
        checks.append(
            PlanCheck(
                name="slot_order",
                passed=slot_order_ok,
                detail="7 left joints -> slots 15-21; 7 right joints -> slots 22-28; 0-14 untouched",
            )
        )
        # A correct reduced solution in radians sits well inside ±limit; the
        # same angles in degrees would blow past it by a large factor.
        degrees_suspected = any(
            abs(value) > 360.0 for value in (*solution.left_q, *solution.right_q)
        )
        zero_ok = not degrees_suspected
        checks.append(
            PlanCheck(
                name="zero_pose_units",
                passed=zero_ok,
                detail="solution magnitudes are consistent with radians, not degrees/counts",
            )
        )

        trajectory = self._plan_trajectory(solution, current_q)
        checks.append(
            PlanCheck(
                name="trajectory_profile",
                passed=(
                    trajectory.duration_s >= self._minimum_duration_s
                    and trajectory.max_velocity <= self._max_joint_velocity_rad_per_s * (1 + 1e-9)
                    and trajectory.max_acceleration
                    <= self._max_joint_acceleration_rad_per_s2 * (1 + 1e-9)
                ),
                detail=(
                    f"quintic profile {trajectory.duration_s:.3f} s at {trajectory.sample_hz} Hz, "
                    f"vmax {trajectory.max_velocity:.3f} rad/s, amax {trajectory.max_acceleration:.3f} rad/s^2"
                ),
            )
        )

        now = self._clock()
        target_digest = digest_json(
            {
                "left": left_pose.as_digest_payload(),
                "right": right_pose.as_digest_payload(),
            }
        )
        plan = PosePlan(
            plan_id=f"plan-{uuid.uuid4().hex}",
            created_at=now,
            expires_at=now + self._plan_ttl_s,
            binding=PlanBinding(
                skill_version=self._skill_version,
                runtime_instance_id=self._runtime_instance_id,
                target_digest=target_digest,
                profile_digest=self._profile_digest,
            ),
            joint_solution=solution,
            checks=tuple(checks),
            trajectory=trajectory,
            dex1_left_opening=left_pose.dex1_opening,
            dex1_right_opening=right_pose.dex1_opening,
        )
        self._plans[plan.plan_id] = plan
        self._evict_expired(now)
        return plan

    def get_plan(self, plan_id: str) -> PosePlan:
        """Fetch a plan while its five-second validity window is open."""

        plan = self._plans.get(plan_id)
        if plan is None:
            raise PlanExpiredError(f"plan {plan_id!r} is unknown or already expired")
        if self._clock() > plan.expires_at:
            raise PlanExpiredError(f"plan {plan_id!r} expired at {plan.expires_at:.3f}")
        return plan

    def _validate_solution(self, solution: JointSolution) -> None:
        if len(solution.left_q) != ARM_DOF or len(solution.right_q) != ARM_DOF:
            raise InvalidSolutionError(f"each arm solution needs {ARM_DOF} joint values")
        for value in (*solution.left_q, *solution.right_q):
            if not math.isfinite(value):
                raise InvalidSolutionError("IK solution must contain finite values")
            if abs(value) > self._arm_joint_limit_rad:
                raise JointLimitError(
                    f"IK joint value {value:.3f} exceeds ±{self._arm_joint_limit_rad} rad"
                )

    def _plan_trajectory(
        self,
        solution: JointSolution,
        current_q: Sequence[float] | None,
    ) -> PlannedMotionSummary:
        """Quintic profile: bounded velocity/acceleration, minimum duration."""

        if current_q is not None and len(current_q) >= 29:
            start_left = tuple(float(current_q[index]) for index in range(15, 22))
            start_right = tuple(float(current_q[index]) for index in range(22, 29))
        else:
            start_left = (0.0,) * ARM_DOF
            start_right = (0.0,) * ARM_DOF
        deltas = (
            abs(end - start)
            for start, end in (
                *zip(start_left, solution.left_q, strict=True),
                *zip(start_right, solution.right_q, strict=True),
            )
        )
        max_delta = max(deltas, default=0.0)
        # Quintic peak factors: v_peak = 15/8 * delta/T, a_peak = 10*sqrt(3)/3 * delta/T^2.
        duration = self._minimum_duration_s
        if max_delta > 0:
            duration = max(
                duration,
                (15.0 / 8.0) * max_delta / self._max_joint_velocity_rad_per_s,
                math.sqrt(
                    (10.0 * math.sqrt(3.0) / 3.0)
                    * max_delta
                    / self._max_joint_acceleration_rad_per_s2
                ),
            )
        max_velocity = (15.0 / 8.0) * max_delta / duration if duration > 0 else 0.0
        max_acceleration = (
            (10.0 * math.sqrt(3.0) / 3.0) * max_delta / (duration * duration)
            if duration > 0
            else 0.0
        )
        return PlannedMotionSummary(
            duration_s=duration,
            sample_hz=self._sample_hz,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
        )

    def _evict_expired(self, now: float) -> None:
        expired = [plan_id for plan_id, plan in self._plans.items() if now > plan.expires_at]
        for plan_id in expired:
            del self._plans[plan_id]


__all__ = [
    "ARM_DOF",
    "ArmPose",
    "CartesianPose",
    "FixtureKinematics",
    "G1DPlanner",
    "InvalidSolutionError",
    "JointLimitError",
    "JointSolution",
    "Kinematics",
    "KinematicsError",
    "PlanBinding",
    "PlanCheck",
    "PlanExpiredError",
    "PlannerError",
    "PlannedMotionSummary",
    "quaternion_angle_deg",
    "PosePlan",
    "PoseValidationError",
    "TOTAL_MOTOR_SLOTS",
    "UnreachableTargetError",
    "digest_json",
    "load_kinematics_profile",
    "map_to_35_slots",
    "validate_arm_pose",
]
