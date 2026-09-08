"""Tests for the pinocchio-based G1_D dual-arm kinematics binding.

The binding is repository-testable: every pinocchio interaction goes through
the injected ``pin`` namespace, so no real robot model or pinocchio install is
needed here.  A scripted fake reproduces the pinocchio API surface the
binding uses (buildModelFromUrdf / buildReducedModel / forwardKinematics /
computeFrameJacobian / log6 / SE3 / Frame) with a planar surrogate model.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from PhyAgentOS.skill_runtime.g1d_kinematics_pin import (
    PinKinematics,
    URDFNotFound,
    default_urdf_path,
)
from PhyAgentOS.skill_runtime.g1d_planner import (
    ArmPose,
    KinematicsError,
    UnreachableTargetError,
)

ARM_JOINT_NAMES = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
FULL_JOINT_NAMES = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"] + ARM_JOINT_NAMES


class Twist:
    def __init__(self, vector):
        self.vector = list(vector)


class FakeSE3:
    def __init__(self, rotation=None, translation=(0.0, 0.0, 0.0)):
        self.rotation = rotation if rotation is not None else _identity3()
        self.translation = list(translation)

    def actInv(self, other):
        # Surrogate: twist-from-frame with translation delta only.
        return FakeSE3(self.rotation,
                       [other.translation[i] - self.translation[i] for i in range(3)])

    def copy(self):
        return FakeSE3([row[:] for row in self.rotation], list(self.translation))


def _identity3():
    return [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]


class FakeModel:
    def __init__(self, joint_names, lower, upper):
        self.names = ["universe"] + joint_names
        self.njoints = len(self.names)
        self.nq = len(joint_names)
        self.lowerPositionLimit = list(lower)
        self.upperPositionLimit = list(upper)
        self._frames: dict[str, str] = {}

    def getJointId(self, name):
        return self.names.index(name)

    def getFrameId(self, name):
        # Surrogate: the frame name is its own frame id.
        return name

    def addFrame(self, frame):
        self._frames[frame.name] = frame.name

    def createData(self):
        return FakeData()


class FakeFrame:
    def __init__(self, name, parent_joint, parent_frame, placement, ftype):
        self.name = name
        self.parent_joint = parent_joint
        self.placement = placement


class FakeData:
    def __init__(self):
        self.oMf: dict[str, FakeSE3] = {}
        self.q: list = []


class FakePin:
    """Scripted pinocchio stand-in with a planar arms-only surrogate model."""

    LOCAL = "local"
    FrameType = type("FrameType", (), {"OP_FRAME": "op"})

    def __init__(self) -> None:
        self.model = FakeModel(FULL_JOINT_NAMES, [-2.8] * len(FULL_JOINT_NAMES),
                               [2.8] * len(FULL_JOINT_NAMES))
        self.built_from = None
        self.locked: list | None = None
        self.reference_used = None

    # -- API used by the binding ------------------------------------------
    def buildModelFromUrdf(self, path):
        self.built_from = str(path)
        return self.model

    def buildReducedModel(self, model, lock, reference):
        self.locked = list(lock)
        # The reduced model keeps exactly the 14 arm joints.
        arm = [n for n in model.names[1:] if n in ARM_JOINT_NAMES]
        self.reference_used = list(reference)
        return FakeModel(arm, [-2.8] * len(arm), [2.8] * len(arm))

    def neutral(self, model):
        return [0.0] * model.nq

    class Frame:  # noqa: N801
        def __init__(self, name, parent_joint, parent_frame, placement, ftype):
            self.name = name
            self.parent_joint = parent_joint
            self.placement = placement
            FakePin.last_frame = self

    def SE3(self, rotation, translation):
        return FakeSE3(rotation if rotation is not None else _identity3(), translation)

    def forwardKinematics(self, model, data, q):
        data.q = list(q)

    def updateFramePlacements(self, model, data):
        # Surrogate FK: left ee position = q[0:3], right ee = q[7:10].
        data.oMf["L_ee"] = FakeSE3(_identity3(), [data.q[0], data.q[1], data.q[2]])
        data.oMf["R_ee"] = FakeSE3(_identity3(), [data.q[7], data.q[8], data.q[9]])

    def computeFrameJacobian(self, model, data, q, fid, ref_frame):
        # Surrogate Jacobian: identity block mapping dq[offset+i] -> dpos[i].
        jacobian = [[0.0] * model.nq for _ in range(6)]
        offset = 0 if fid == "L_ee" else 7
        for i in range(3):
            jacobian[i][offset + i] = 1.0
        return jacobian

    def log6(self, se3):
        # 6d twist: translation delta + zero rotation.
        return Twist([se3.translation[i] for i in range(3)] + [0.0, 0.0, 0.0])


def make_kinematics(pin=None, **kwargs):
    pin = pin or FakePin()
    return PinKinematics(pin=pin, urdf_path=Path("/fake/g1_d.urdf"), **kwargs), pin


def arm_pose(position):
    return ArmPose(frame_id="g1d_base", position_m=position,
                   orientation_xyzw=(0.0, 0.0, 0.0, 1.0))


# ---------------------------------------------------------------------------
# Model construction
# ---------------------------------------------------------------------------


def test_default_urdf_path_points_at_vendored_asset() -> None:
    path = default_urdf_path()
    assert path.name == "g1_d.urdf"
    assert "assets" in str(path)


def test_pin_kinematics_builds_reduced_model_with_ee_frames() -> None:
    kin, pin = make_kinematics()

    assert pin.built_from == "/fake/g1_d.urdf"
    # Waist joints locked, arm joints kept.
    assert pin.locked == [1, 2, 3]  # universe=0, waist ids 1..3
    # EE frames appended on the reduced model.
    assert set(kin.model._frames) >= {"L_ee", "R_ee"}
    assert kin.nq == 14


def test_pin_kinematics_missing_urdf_is_explicit() -> None:
    pin = FakePin()

    def _missing(path):
        raise FileNotFoundError(path)

    pin.buildModelFromUrdf = _missing
    with pytest.raises(URDFNotFound, match="g1_d"):
        PinKinematics(pin=pin, urdf_path=Path("/fake/g1_d.urdf"))


# ---------------------------------------------------------------------------
# IK / FK round trip against the surrogate model
# ---------------------------------------------------------------------------


def test_solve_ik_converges_and_fk_round_trips() -> None:
    kin, pin = make_kinematics()

    solution = kin.solve_ik(arm_pose((0.1, 0.2, 0.3)), arm_pose((0.1, -0.2, 0.3)))

    assert len(solution.left_q) == 7 and len(solution.right_q) == 7
    # Surrogate: left wrist q[0:3] equals the target position.
    assert solution.left_q[:3] == pytest.approx((0.1, 0.2, 0.3), abs=1e-3)
    assert solution.right_q[:3] == pytest.approx((0.1, -0.2, 0.3), abs=1e-3)
    # FK agrees.
    fk_left, fk_right = kin.solve_fk(solution.left_q, solution.right_q)
    assert fk_left.position_m == pytest.approx((0.1, 0.2, 0.3), abs=1e-3)
    assert fk_right.position_m == pytest.approx((0.1, -0.2, 0.3), abs=1e-3)


def test_solve_ik_starts_from_reference_q_when_provided() -> None:
    kin, pin = make_kinematics()
    reference = [0.05] * 14

    kin.set_reference_q(reference)
    solution = kin.solve_ik(arm_pose((0.1, 0.2, 0.3)), arm_pose((0.1, -0.2, 0.3)))

    # The reference seeds the solve, so joints the task does not use stay at
    # their reference value.
    assert solution.left_q[3:] == pytest.approx([0.05] * 4, abs=0.05)
    assert solution.right_q[3:] == pytest.approx([0.05] * 4, abs=0.05)


def test_solve_ik_unreachable_target_raises() -> None:
    kin, pin = make_kinematics()
    # Beyond the surrogate's reachable set (joint limit 2.8).
    with pytest.raises(UnreachableTargetError):
        kin.solve_ik(arm_pose((5.0, 5.0, 5.0)), arm_pose((0.1, -0.2, 0.3)))


def test_solve_ik_satisfies_joint_limits() -> None:
    kin, pin = make_kinematics()
    solution = kin.solve_ik(arm_pose((0.1, 0.2, 0.3)), arm_pose((0.1, -0.2, 0.3)))
    for q in (*solution.left_q, *solution.right_q):
        assert -2.8 <= q <= 2.8


def test_solve_fk_rejects_wrong_joint_counts() -> None:
    kin, _ = make_kinematics()
    with pytest.raises(KinematicsError, match="7"):
        kin.solve_fk([0.0] * 6, [0.0] * 7)
    with pytest.raises(KinematicsError, match="7"):
        kin.solve_fk([0.0] * 7, [0.0] * 8)


# ---------------------------------------------------------------------------
# Solver diagnostics
# ---------------------------------------------------------------------------


def test_solver_reports_last_iteration_count_and_residual() -> None:
    kin, _ = make_kinematics()
    kin.solve_ik(arm_pose((0.1, 0.2, 0.3)), arm_pose((0.1, -0.2, 0.3)))
    stats = kin.last_stats
    assert stats["iterations"] >= 1
    assert stats["residual"] < 1e-3
