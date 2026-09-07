from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from PhyAgentOS.skill_runtime.g1d_adapter import make_lowstate_frame
from PhyAgentOS.skill_runtime.g1d_planner import (
    FixtureKinematics,
    G1DPlanner,
    InvalidSolutionError,
    JointLimitError,
    JointSolution,
    PlanBinding,
    PlanExpiredError,
    PoseValidationError,
    UnreachableTargetError,
    digest_json,
    load_kinematics_profile,
    map_to_35_slots,
    validate_arm_pose,
)

BUNDLE = Path(__file__).parents[1] / "bundles" / "g1d-manipulation"
KINEMATICS = BUNDLE / "profiles" / "real-g1d" / "kinematics.json"

HOME_LEFT = {
    "frame_id": "g1d_base",
    "position_m": [0.05, 0.25, 0.10],
    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
}
HOME_RIGHT = {
    "frame_id": "g1d_base",
    "position_m": [0.05, -0.25, 0.10],
    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
}
HOME_LEFT_Q = [0.05, 0.25, 0.10, 0.0, 0.0, 0.0, 1.0]
HOME_RIGHT_Q = [0.05, -0.25, 0.10, 0.0, 0.0, 0.0, 1.0]


class Clock:
    def __init__(self) -> None:
        self.value = 1000.0

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def make_planner(
    clock: Clock,
    fixtures: list | None = None,
    **overrides,
) -> G1DPlanner:
    profile = load_kinematics_profile(KINEMATICS)
    params = {**profile, **overrides}
    kinematics = FixtureKinematics(
        fixtures=fixtures
        if fixtures is not None
        else [(dict(HOME_LEFT), dict(HOME_RIGHT), HOME_LEFT_Q, HOME_RIGHT_Q)]
    )
    return G1DPlanner(
        kinematics=kinematics,
        clock=clock,
        skill_version="0.1.0",
        runtime_instance_id="runtime-test",
        profile_digest=digest_json(json.loads(KINEMATICS.read_text())),
        **{
            key: value
            for key, value in params.items()
            if key
            in {
                "arm_joint_limit_rad",
                "max_joint_velocity_rad_per_s",
                "max_joint_acceleration_rad_per_s2",
                "minimum_duration_s",
                "sample_hz",
                "position_tolerance_m",
                "orientation_tolerance_deg",
            }
        },
    )


class TestPoseValidation:
    def test_valid_pose_is_accepted_in_meters_and_normalized_xyzw(self) -> None:
        pose = validate_arm_pose(HOME_LEFT)
        assert pose.frame_id == "g1d_base"
        assert pose.position_m == (0.05, 0.25, 0.10)
        assert pose.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
        assert pose.dex1_opening is None

    def test_missing_or_empty_frame_id_is_rejected(self) -> None:
        with pytest.raises(PoseValidationError, match="frame_id"):
            validate_arm_pose({**HOME_LEFT, "frame_id": ""})

    def test_wrong_position_shape_is_rejected(self) -> None:
        with pytest.raises(PoseValidationError, match="position_m"):
            validate_arm_pose({**HOME_LEFT, "position_m": [0.1, 0.2]})

    def test_non_finite_values_are_rejected(self) -> None:
        with pytest.raises(PoseValidationError, match="finite"):
            validate_arm_pose({**HOME_LEFT, "position_m": [float("nan"), 0.2, 0.3]})

    def test_denormalized_quaternion_is_rejected(self) -> None:
        with pytest.raises(PoseValidationError, match="normal"):
            validate_arm_pose({**HOME_LEFT, "orientation_xyzw": [0.0, 0.0, 0.0, 2.0]})

    def test_non_quaternion_orientation_length_is_rejected(self) -> None:
        with pytest.raises(PoseValidationError, match="orientation_xyzw"):
            validate_arm_pose({**HOME_LEFT, "orientation_xyzw": [0.0, 0.0, 1.0]})

    def test_unreasonable_position_magnitude_is_rejected_as_unit_violation(self) -> None:
        # A position of 500 in "metres" is outside any G1_D arm envelope:
        # the units check must reject it before IK ever runs.
        with pytest.raises(PoseValidationError, match="reach envelope"):
            validate_arm_pose({**HOME_LEFT, "position_m": [500.0, 0.2, 0.3]})

    def test_dex1_opening_is_validated(self) -> None:
        pose = validate_arm_pose({**HOME_LEFT, "dex1": {"opening": 0.5}})
        assert pose.dex1_opening == 0.5
        with pytest.raises(PoseValidationError, match="dex1"):
            validate_arm_pose({**HOME_LEFT, "dex1": {"opening": 2.0}})


class TestFixtureKinematics:
    def test_recorded_fixtures_solve_deterministically(self) -> None:
        kinematics = FixtureKinematics(
            fixtures=[(dict(HOME_LEFT), dict(HOME_RIGHT), HOME_LEFT_Q, HOME_RIGHT_Q)]
        )
        solution = kinematics.solve_ik(validate_arm_pose(HOME_LEFT), validate_arm_pose(HOME_RIGHT))
        assert solution.left_q == tuple(HOME_LEFT_Q)
        assert solution.right_q == tuple(HOME_RIGHT_Q)

    def test_unrecorded_pose_is_explicitly_unreachable(self) -> None:
        kinematics = FixtureKinematics(
            fixtures=[(dict(HOME_LEFT), dict(HOME_RIGHT), HOME_LEFT_Q, HOME_RIGHT_Q)]
        )
        far = {**HOME_LEFT, "position_m": [0.4, 0.25, 0.10]}
        with pytest.raises(UnreachableTargetError):
            kinematics.solve_ik(validate_arm_pose(far), validate_arm_pose(HOME_RIGHT))

    def test_fk_round_trips_recorded_targets(self) -> None:
        kinematics = FixtureKinematics(
            fixtures=[(dict(HOME_LEFT), dict(HOME_RIGHT), HOME_LEFT_Q, HOME_RIGHT_Q)]
        )
        left, right = kinematics.solve_fk(tuple(HOME_LEFT_Q), tuple(HOME_RIGHT_Q))
        assert left.position_m == pytest.approx(HOME_LEFT["position_m"])
        assert right.position_m == pytest.approx(HOME_RIGHT["position_m"])


class TestSlotMapping:
    def test_left_maps_to_15_21_right_to_22_28_and_0_14_preserved(self) -> None:
        preserved = [float(index) for index in range(15)]
        solution = JointSolution(
            left_q=(1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6),
            right_q=(2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6),
        )
        slots = map_to_35_slots(solution, preserved)

        assert len(slots) == 35
        assert slots[:15] == tuple(float(index) for index in range(15))
        assert slots[15:22] == solution.left_q
        assert slots[22:29] == solution.right_q
        # The HG frame carries 35 slots; the G1_D has 29 valid motors.
        assert slots[29:] == (0.0,) * 6

    def test_preserved_slots_must_be_complete(self) -> None:
        with pytest.raises(ValueError, match="15"):
            map_to_35_slots(JointSolution(left_q=(0.0,) * 7, right_q=(0.0,) * 7), [0.0] * 14)

    def test_profile_slots_match_the_mapping(self) -> None:
        profile = load_kinematics_profile(KINEMATICS)
        assert profile["left_slots"] == list(range(15, 22))
        assert profile["right_slots"] == list(range(22, 29))
        assert profile["preserved_slots"] == list(range(15))


class TestPlanner:
    def test_plan_pose_returns_identity_binding_and_checks(self) -> None:
        clock = Clock()
        planner = make_planner(clock)

        plan = planner.plan_pose(left=dict(HOME_LEFT), right=dict(HOME_RIGHT), current_q=[0.0] * 35)

        assert plan.checks_passed
        assert {check.name for check in plan.checks} >= {
            "reachability",
            "fk_ik_roundtrip",
            "slot_order",
            "joint_limits",
            "units_finite",
            "zero_pose_units",
            "trajectory_profile",
        }
        assert plan.binding.skill_version == "0.1.0"
        assert plan.binding.runtime_instance_id == "runtime-test"
        assert plan.binding.target_digest
        assert plan.binding.profile_digest
        assert plan.expires_at - plan.created_at == pytest.approx(5.0)

    def test_identical_targets_share_digest_but_get_distinct_plan_ids(self) -> None:
        clock = Clock()
        planner = make_planner(clock)
        first = planner.plan_pose(
            left=dict(HOME_LEFT), right=dict(HOME_RIGHT), current_q=[0.0] * 35
        )
        second = planner.plan_pose(
            left=dict(HOME_LEFT), right=dict(HOME_RIGHT), current_q=[0.0] * 35
        )
        assert first.binding.target_digest == second.binding.target_digest
        assert first.plan_id != second.plan_id

    def test_plan_expires_after_five_seconds(self) -> None:
        clock = Clock()
        planner = make_planner(clock)
        plan = planner.plan_pose(left=dict(HOME_LEFT), right=dict(HOME_RIGHT), current_q=[0.0] * 35)

        clock.advance(4.9)
        assert planner.get_plan(plan.plan_id).plan_id == plan.plan_id
        clock.advance(0.2)
        with pytest.raises(PlanExpiredError, match="expired"):
            planner.get_plan(plan.plan_id)

    def test_unreachable_target_is_rejected(self) -> None:
        clock = Clock()
        planner = make_planner(clock)
        far = {**HOME_LEFT, "position_m": [0.4, 0.25, 0.10]}
        with pytest.raises(UnreachableTargetError):
            planner.plan_pose(left=far, right=dict(HOME_RIGHT), current_q=[0.0] * 35)

    def test_fk_ik_roundtrip_failure_is_a_check_not_a_crash(self) -> None:
        clock = Clock()
        drifted = {**HOME_LEFT, "position_m": [0.10, 0.25, 0.10]}
        # Record a fixture whose FK does not reproduce the requested pose.
        planner = make_planner(
            clock,
            fixtures=[
                (dict(HOME_LEFT), dict(HOME_RIGHT), HOME_LEFT_Q, HOME_RIGHT_Q),
                (dict(drifted), dict(HOME_RIGHT), HOME_LEFT_Q, HOME_RIGHT_Q),
            ],
        )
        plan = planner.plan_pose(left=dict(drifted), right=dict(HOME_RIGHT), current_q=[0.0] * 35)
        roundtrip = next(c for c in plan.checks if c.name == "fk_ik_roundtrip")
        assert not roundtrip.passed
        assert not plan.checks_passed

    def test_joint_limits_are_enforced(self) -> None:
        clock = Clock()
        out_of_range_q = [2.9, 0.25, 0.10, 0.0, 0.0, 0.0, 1.0]
        planner = make_planner(
            clock,
            fixtures=[(dict(HOME_LEFT), dict(HOME_RIGHT), out_of_range_q, HOME_RIGHT_Q)],
        )
        with pytest.raises(JointLimitError, match="exceeds"):
            planner.plan_pose(left=dict(HOME_LEFT), right=dict(HOME_RIGHT), current_q=[0.0] * 35)

    def test_non_finite_ik_output_is_rejected(self) -> None:
        clock = Clock()
        planner = make_planner(
            clock,
            fixtures=[
                (
                    dict(HOME_LEFT),
                    dict(HOME_RIGHT),
                    [float("inf")] + HOME_LEFT_Q[1:],
                    HOME_RIGHT_Q,
                )
            ],
        )
        with pytest.raises(InvalidSolutionError, match="finite"):
            planner.plan_pose(left=dict(HOME_LEFT), right=dict(HOME_RIGHT), current_q=[0.0] * 35)

    def test_trajectory_profile_respects_minimum_duration_and_caps(self) -> None:
        clock = Clock()
        planner = make_planner(clock, minimum_duration_s=1.0)
        plan = planner.plan_pose(left=dict(HOME_LEFT), right=dict(HOME_RIGHT), current_q=[0.0] * 35)
        trajectory = plan.trajectory
        assert trajectory.duration_s >= 1.0
        assert trajectory.sample_hz == 500
        assert trajectory.max_velocity <= planner.max_joint_velocity_rad_per_s * (1 + 1e-6)
        assert trajectory.max_acceleration <= planner.max_joint_acceleration_rad_per_s2 * (1 + 1e-6)

    def test_slot_command_from_state_preserves_0_14(self) -> None:
        clock = Clock()
        planner = make_planner(clock)
        frame = make_lowstate_frame(
            mode_machine=7,
            tick=42,
            positions=[float(index) for index in range(35)],
        )
        plan = planner.plan_pose(
            left=dict(HOME_LEFT), right=dict(HOME_RIGHT), current_q=frame.positions
        )
        slots = map_to_35_slots(plan.joint_solution, frame.positions[:15])
        assert slots[:15] == tuple(float(index) for index in range(15))
        assert slots[15:22] == tuple(HOME_LEFT_Q)
        assert slots[22:29] == tuple(HOME_RIGHT_Q)


class TestDigest:
    def test_digest_is_order_insensitive_for_mappings(self) -> None:
        assert digest_json({"a": 1, "b": 2}) == digest_json({"b": 2, "a": 1})
        assert digest_json({"a": 1}) != digest_json({"a": 2})

    def test_binding_digests_are_hex_sha256(self) -> None:
        clock = Clock()
        planner = make_planner(clock)
        plan = planner.plan_pose(left=dict(HOME_LEFT), right=dict(HOME_RIGHT), current_q=[0.0] * 35)
        for digest in (plan.binding.target_digest, plan.binding.profile_digest):
            assert len(digest) == 64
            int(digest, 16)


def test_pose_plan_identity_uses_binding_digests() -> None:
    clock = Clock()
    planner = make_planner(clock)
    plan = planner.plan_pose(left=dict(HOME_LEFT), right=dict(HOME_RIGHT), current_q=[0.0] * 35)
    assert isinstance(plan.binding, PlanBinding)
    assert plan.binding.target_digest == digest_json(
        {
            "left": {
                "frame_id": "g1d_base",
                "position_m": [0.05, 0.25, 0.10],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "dex1_opening": None,
            },
            "right": {
                "frame_id": "g1d_base",
                "position_m": [0.05, -0.25, 0.10],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "dex1_opening": None,
            },
        }
    )
    assert math.isfinite(plan.expires_at)
