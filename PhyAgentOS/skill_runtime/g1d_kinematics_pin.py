"""Self-contained G1_D dual-arm kinematics on pinocchio.

This module binds the planner's :class:`Kinematics` protocol to a
pinocchio-reduced 14-DoF arm model built from the vendored G1_D URDF
(``assets/robots/g1d/g1_d.urdf``).  It deliberately depends on nothing from
UniRobot or the Unitree SDK: the model construction, end-effector frames, and
the damped-least-squares IK are implemented here.

Design:

- The full URDF model is reduced to the 14 arm joints (waist and finger
  joints locked at neutral); the reduced joint order matches the motor slot
  order 15-28 (left 7 then right 7), which the adapter maps onto wire slots.
- End-effector frames ``L_ee`` / ``R_ee`` sit 5 cm beyond the wrist-yaw joint
  along the forearm axis (the Dex1 palm mounting offset).
- IK is iterative damped least squares (Local frame 6-D twist error) with a
  weak posture pull toward the reference configuration and joint-limit
  clamping.  The last accepted solution doubles as the next warm start.
- pinocchio is imported lazily; repository tests inject a scripted fake, and
  importing this module never requires the robotics stack.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Sequence

from PhyAgentOS.skill_runtime.g1d_planner import (
    ARM_DOF,
    ArmPose,
    CartesianPose,
    JointSolution,
    KinematicsError,
    UnreachableTargetError,
)

#: Arm joints of the reduced model, in motor-slot order (15..28).
ARM_JOINT_NAMES: tuple[str, ...] = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)

#: End-effector frame names on the reduced model.
EE_FRAME_NAMES = ("L_ee", "R_ee")

#: Wrist joint each EE frame is attached to.
EE_PARENT_JOINTS = ("left_wrist_yaw_joint", "right_wrist_yaw_joint")

#: Dex1 palm mounting offset beyond the wrist-yaw joint, metres.
EE_TRANSLATION_M = (0.05, 0.0, 0.0)

# IK tuning defaults.
DEFAULT_MAX_ITERATIONS = 100
DEFAULT_TOLERANCE = 1e-4  # ~0.1 mm + orientation equivalent
DEFAULT_DAMPING = 1e-4
DEFAULT_POSTURE_GAIN = 1e-3


class KinematicsInitError(KinematicsError):
    """The pinocchio model could not be built."""


class URDFNotFound(KinematicsInitError):  # noqa: N818 - retained public exception name
    """The configured URDF file does not exist."""


def default_urdf_path() -> Path:
    """Vendored G1_D URDF shipped with the repository."""

    return Path(__file__).resolve().parents[2] / "assets" / "robots" / "g1d" / "g1_d.urdf"


def _import_pinocchio() -> Any:
    try:
        import pinocchio  # type: ignore[import-untyped]
    except ImportError as error:  # pragma: no cover - environment dependent
        raise KinematicsInitError(
            "pinocchio is not importable in this environment; install `pin` "
            "or inject pin= for tests"
        ) from error
    return pinocchio


# ---------------------------------------------------------------------------
# Pure-python quaternion / rotation helpers (no pinocchio dependency)
# ---------------------------------------------------------------------------


def _normalize_quaternion(xyzw: Sequence[float]) -> tuple[float, float, float, float]:
    x, y, z, w = (float(value) for value in xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("orientation_xyzw must be a finite non-zero quaternion")
    return x / norm, y / norm, z / norm, w / norm


def quaternion_to_matrix(xyzw: Sequence[float]) -> list[list[float]]:
    """3x3 rotation matrix from a normalized xyzw quaternion."""

    x, y, z, w = _normalize_quaternion(xyzw)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def matrix_to_quaternion(rotation: Sequence[Sequence[float]]) -> tuple[float, float, float, float]:
    """Normalized xyzw quaternion from a 3x3 rotation matrix."""

    r = [[float(rotation[i][j]) for j in range(3)] for i in range(3)]
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s

    x, y, z, w = _normalize_quaternion((qx, qy, qz, qw))
    if w < 0.0:  # canonical sign
        x, y, z, w = -x, -y, -z, -w
    return x, y, z, w


def _numpy():
    """Lazy numpy import (only IK/FK math needs it)."""

    import numpy  # noqa: PLC0415

    return numpy


def _target_se3(pin: Any, pose: ArmPose) -> Any:
    """SE3 target for one arm from the skill's xyzw pose vocabulary."""

    np = _numpy()
    rotation = np.asarray(quaternion_to_matrix(pose.orientation_xyzw), dtype=float)
    translation = np.asarray(pose.position_m, dtype=float)
    return pin.SE3(rotation, translation)


class PinKinematics:
    """Dual-arm IK/FK on a pinocchio-reduced 14-DoF G1_D model."""

    def __init__(
        self,
        *,
        pin: Any = None,
        urdf_path: Path | str | None = None,
        max_iterations: int = DEFAULT_MAX_ITERATIONS,
        tolerance: float = DEFAULT_TOLERANCE,
        damping: float = DEFAULT_DAMPING,
        posture_gain: float = DEFAULT_POSTURE_GAIN,
    ) -> None:
        self._pin = pin if pin is not None else _import_pinocchio()
        self._max_iterations = int(max_iterations)
        self._tolerance = float(tolerance)
        self._damping = float(damping)
        self._posture_gain = float(posture_gain)

        if urdf_path is None:
            path = default_urdf_path()
            if not path.is_file():
                raise URDFNotFound(f"G1_D URDF not found at {path}")
        else:
            path = Path(urdf_path)
        self._urdf_path = path

        self._build_model()
        self._reference_q = [0.0] * self.nq
        self._last_solution: list[float] | None = None
        self.last_stats: dict[str, float | int] = {}

    # -- model construction -------------------------------------------------

    def _build_model(self) -> None:
        pin = self._pin
        try:
            full = pin.buildModelFromUrdf(str(self._urdf_path))
        except FileNotFoundError as error:
            raise URDFNotFound(f"G1_D URDF not found at {self._urdf_path}") from error

        arm_names = set(ARM_JOINT_NAMES)
        lock_ids = [
            joint_id
            for joint_id in range(1, full.njoints)
            if full.names[joint_id] not in arm_names
        ]
        self.model = pin.buildReducedModel(full, lock_ids, pin.neutral(full))
        model = self.model

        actual = [model.names[j] for j in range(1, model.njoints)]
        if tuple(actual) != ARM_JOINT_NAMES:
            raise KinematicsInitError(
                f"reduced model joint order {actual} does not match the "
                "expected 14-joint arm order (left 7 then right 7)"
            )

        for name, parent in zip(EE_FRAME_NAMES, EE_PARENT_JOINTS):
            np = _numpy()
            placement = pin.SE3(
                np.asarray(quaternion_to_matrix((0.0, 0.0, 0.0, 1.0)), dtype=float),
                np.asarray(EE_TRANSLATION_M, dtype=float),
            )
            model.addFrame(
                pin.Frame(
                    name,
                    model.getJointId(parent),
                    model.getFrameId(parent),
                    placement,
                    pin.FrameType.OP_FRAME,
                )
            )

        self._data = model.createData()
        self._frame_ids = {name: model.getFrameId(name) for name in EE_FRAME_NAMES}

    @property
    def nq(self) -> int:
        return int(self.model.nq)

    # -- Kinematics protocol --------------------------------------------------

    def set_reference_q(self, reference: Sequence[float]) -> None:
        """Seed the next solve (and the posture pull) from a robot state."""

        reference = [float(value) for value in reference]
        if len(reference) != self.nq:
            raise KinematicsError(
                f"reference needs {self.nq} joint values, got {len(reference)}"
            )
        self._reference_q = reference
        self._last_solution = None

    def solve_ik(self, left: ArmPose, right: ArmPose) -> JointSolution:
        import numpy as np  # local: only the solver itself needs linear algebra

        pin = self._pin
        model, data = self.model, self._data
        if self._last_solution is not None:
            q = np.asarray(self._last_solution, dtype=float)
        else:
            q = np.asarray(self._reference_q, dtype=float)

        targets = {
            "L_ee": _target_se3(pin, left),
            "R_ee": _target_se3(pin, right),
        }
        lower = np.asarray(model.lowerPositionLimit, dtype=float)
        upper = np.asarray(model.upperPositionLimit, dtype=float)
        damping_eye = self._damping * np.eye(12)
        residual = math.inf
        iterations = 0

        for iterations in range(1, self._max_iterations + 1):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            errors = [
                pin.log6(data.oMf[self._frame_ids[name]].actInv(target)).vector
                for name, target in targets.items()
            ]
            e = np.concatenate([np.asarray(err, dtype=float) for err in errors])
            residual = float(np.linalg.norm(e))
            if residual < self._tolerance:
                break
            jacobians = [
                pin.computeFrameJacobian(
                    model, data, q, self._frame_ids[name], pin.LOCAL
                )
                for name in EE_FRAME_NAMES
            ]
            jacobian = np.vstack([np.asarray(j, dtype=float) for j in jacobians])
            inverse = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping_eye, np.eye(12))
            dq_task = inverse @ e
            dq_posture = self._posture_gain * (np.eye(self.nq) - inverse @ jacobian) @ (
                np.asarray(self._reference_q, dtype=float) - np.asarray(q, dtype=float)
            )
            q = np.clip(
                np.asarray(q, dtype=float) + dq_task + dq_posture, lower, upper
            )

        self.last_stats = {"iterations": iterations, "residual": residual}
        if residual >= self._tolerance:
            self._last_solution = None  # do not warm-start from a failure
            raise UnreachableTargetError(
                f"IK did not converge within {self._max_iterations} iterations "
                f"(residual {residual:.4f})"
            )

        solution = [float(value) for value in q]
        self._last_solution = solution
        left_q, right_q = solution[:ARM_DOF], solution[ARM_DOF:]
        return JointSolution(left_q=tuple(left_q), right_q=tuple(right_q))

    def solve_fk(
        self, left_q: Sequence[float], right_q: Sequence[float]
    ) -> tuple[CartesianPose, CartesianPose]:
        if len(left_q) != ARM_DOF or len(right_q) != ARM_DOF:
            raise KinematicsError(f"each arm needs {ARM_DOF} joint values")
        for value in (*left_q, *right_q):
            if not math.isfinite(value):
                raise KinematicsError("joint values must be finite")

        pin = self._pin
        model, data = self.model, self._data
        q = _numpy().asarray([*left_q, *right_q], dtype=float)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        poses = []
        for name in EE_FRAME_NAMES:
            placement = data.oMf[self._frame_ids[name]]
            poses.append(
                CartesianPose(
                    position_m=(float(placement.translation[0]), float(placement.translation[1]), float(placement.translation[2])),
                    orientation_xyzw=matrix_to_quaternion(placement.rotation),
                )
            )
        return poses[0], poses[1]


__all__ = [
    "ARM_JOINT_NAMES",
    "DEFAULT_DAMPING",
    "DEFAULT_MAX_ITERATIONS",
    "DEFAULT_POSTURE_GAIN",
    "DEFAULT_TOLERANCE",
    "EE_FRAME_NAMES",
    "EE_PARENT_JOINTS",
    "EE_TRANSLATION_M",
    "KinematicsInitError",
    "PinKinematics",
    "URDFNotFound",
    "default_urdf_path",
    "matrix_to_quaternion",
    "quaternion_to_matrix",
]
