"""Acceptance with the real numerical model, never attached to hardware."""

from dataclasses import asdict

import pytest

from PhyAgentOS.skill_runtime.g1d_kinematics_pin import PinKinematics, default_urdf_path
from PhyAgentOS.skill_runtime.g1d_planner import (
    G1DPlanner,
    JointLimitError,
    PoseValidationError,
    UnreachableTargetError,
)


def real_planner():
    kinematics = PinKinematics()
    return kinematics, G1DPlanner(
        kinematics=kinematics,
        clock=lambda: 100.0,
        skill_version="0.1.0",
        runtime_instance_id="real-model-test",
        profile_digest="test",
        joint_limits_rad=list(
            zip(kinematics.model.lowerPositionLimit, kinematics.model.upperPositionLimit)
        ),
        base_frame="g1d_base",
        require_current_state=True,
    )


def test_real_planner_requires_valid_state_and_declared_frame():
    kinematics, planner = real_planner()
    left, right = kinematics.solve_fk([0.0] * 7, [0.0] * 7)
    targets = {
        "left": {"frame_id": "g1d_base", **asdict(left)},
        "right": {"frame_id": "g1d_base", **asdict(right)},
    }
    with pytest.raises(PoseValidationError, match="current"):
        planner.plan_pose(**targets)
    current = [0.0] * 35
    current[20] = 1.7  # URDF wrist pitch upper limit is 1.614429558.
    with pytest.raises(JointLimitError):
        planner.plan_pose(**targets, current_q=current)
    for target in targets.values():
        target["frame_id"] = "world"
    with pytest.raises(PoseValidationError, match="frame"):
        planner.plan_pose(**targets, current_q=[0.0] * 35)


def test_real_model_can_plan_current_bilateral_pose():
    kinematics = PinKinematics()
    left, right = kinematics.solve_fk([0.0] * 7, [0.0] * 7)
    planner = G1DPlanner(
        kinematics=kinematics,
        clock=lambda: 100.0,
        skill_version="0.2.0",
        runtime_instance_id="real-model-test",
        profile_digest="test",
    )
    plan = planner.plan_pose(
        left={"frame_id": "g1d_base", **asdict(left)},
        right={"frame_id": "g1d_base", **asdict(right)},
        current_q=[0.0] * 35,
    )
    assert plan.checks_passed
    assert plan.joint_solution.left_q == pytest.approx([0.0] * 7)
    assert plan.joint_solution.right_q == pytest.approx([0.0] * 7)


@pytest.mark.parametrize(
    "q",
    [
        [0.15, 0.2, -0.1, 0.35, 0.1, -0.15, 0.1, -0.2, -0.1, 0.15, 0.25, -0.1, 0.1, -0.15],
        [0, 0, 0, 0, 0, 1.6, 0, 0, 0, 0, 0, 0, -1.6, 0],
    ],
)
def test_real_planner_roundtrips_asymmetric_and_near_limit_targets(q):
    kinematics, planner = real_planner()
    left, right = kinematics.solve_fk(q[:7], q[7:])
    plan = planner.plan_pose(
        left={"frame_id": "g1d_base", **asdict(left)},
        right={"frame_id": "g1d_base", **asdict(right)},
        current_q=[0.0] * 35,
    )
    assert plan.checks_passed
    actual_left, actual_right = kinematics.solve_fk(
        plan.joint_solution.left_q, plan.joint_solution.right_q
    )
    assert actual_left.position_m == pytest.approx(left.position_m, abs=1e-4)
    assert actual_right.position_m == pytest.approx(right.position_m, abs=1e-4)


def reference_pose(side, angles):
    """Independent URDF tree traversal using homogeneous transforms, no Pinocchio."""
    import math
    import xml.etree.ElementTree as ET

    import numpy as np

    def rotate(axis, angle):
        x, y, z = axis
        skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew

    model = ET.parse(default_urdf_path()).getroot()
    by_child = {joint.find("child").get("link"): joint for joint in model.findall("joint")}
    arm_names = [
        f"{side}_{name}_joint"
        for name in (
            "shoulder_pitch",
            "shoulder_roll",
            "shoulder_yaw",
            "elbow",
            "wrist_roll",
            "wrist_pitch",
            "wrist_yaw",
        )
    ]
    q = dict(zip(arm_names, angles, strict=True))
    link = f"{side}_wrist_yaw_link"
    chain = []
    while link in by_child:
        joint = by_child[link]
        chain.append(joint)
        link = joint.find("parent").get("link")
    assert link == "pelvis"
    transform = np.eye(4)
    for joint in reversed(chain):
        origin = joint.find("origin")
        xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        r, p, y = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        local = np.eye(4)
        local[:3, :3] = rotate((0, 0, 1), y) @ rotate((0, 1, 0), p) @ rotate((1, 0, 0), r)
        local[:3, 3] = xyz
        axis = joint.find("axis")
        if axis is not None:
            local[:3, :3] = local[:3, :3] @ rotate(
                [float(v) for v in axis.get("xyz").split()], q.get(joint.get("name"), 0)
            )
        transform = transform @ local
    return transform[:3, :3], (transform @ np.array([0.05, 0, 0, 1]))[:3]


def test_planner_matches_independent_urdf_chain_and_each_joint_sign():
    import numpy as np

    from PhyAgentOS.skill_runtime.g1d_kinematics_pin import quaternion_to_matrix

    kinematics, planner = real_planner()
    assert kinematics.nq == 14
    q = [0.15, 0.2, -0.1, 0.35, 0.1, -0.15, 0.1, -0.2, -0.1, 0.15, 0.25, -0.1, 0.1, -0.15]
    for index in range(14):
        moved = q.copy()
        moved[index] += 0.1
        poses = kinematics.solve_fk(moved[:7], moved[7:])
        for side, arm, pose in zip(("left", "right"), (moved[:7], moved[7:]), poses):
            rotation, position = reference_pose(side, arm)
            assert pose.position_m == pytest.approx(position, abs=1e-10)
            assert np.asarray(quaternion_to_matrix(pose.orientation_xyzw)) == pytest.approx(
                rotation, abs=1e-10
            )
    # Independently calculated neutral chain reference, including +5 cm TCP mount.
    left, right = kinematics.solve_fk([0.0] * 7, [0.0] * 7)
    assert left.position_m == pytest.approx(
        (0.249774283850806, 0.148652124666052, 0.095230084295346), abs=1e-12
    )
    assert right.position_m == pytest.approx(
        (0.249774283850806, -0.148642124666052, 0.095230084295346), abs=1e-12
    )
    impossible = {
        "frame_id": "g1d_base",
        "position_m": [1.1, 1.1, 1.1],
        "orientation_xyzw": [0, 0, 0, 1],
    }
    with pytest.raises(UnreachableTargetError):
        planner.plan_pose(left=impossible, right=impossible, current_q=[0.0] * 35)
