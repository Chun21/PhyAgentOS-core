"""Independent runtime of the pinned UniRobot G1_D_DEX1_ArmIK algorithm.

Objective, IPOPT options and float32 filtering follow UniRobot (Apache-2.0).
The exported CasADi graphs contain its actual reduced model, including inertia.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Sequence

from PhyAgentOS.skill_runtime.g1d_kinematics_pin import (
    ARM_JOINT_NAMES,
    matrix_to_quaternion,
    quaternion_to_matrix,
)
from PhyAgentOS.skill_runtime.g1d_planner import (
    ArmPose,
    CartesianPose,
    JointSolution,
    KinematicsError,
    UnreachableTargetError,
)


def model_directory() -> Path:
    return Path(__file__).resolve().parents[2] / "assets/robots/g1d/unirobot_ik"


class UniRobotKinematics:
    """14-joint fixed-base IK; waist is outside this model's target frame."""

    def __init__(self, *, directory: Path | None = None) -> None:
        import casadi as ca
        import numpy as np

        if ca.__version__ != "3.6.7":
            raise KinematicsError("UniRobot IK requires CasADi 3.6.7")
        directory = directory or model_directory()
        metadata = json.loads((directory / "model.json").read_text())
        if tuple(metadata["joint_names"]) != ARM_JOINT_NAMES:
            raise KinematicsError("reference model joint order mismatch")
        functions = {}
        for filename, digest in metadata["functions"].items():
            path = directory / filename
            if path.parent != directory or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise KinematicsError("reference model graph hash mismatch")
            functions[path.stem] = ca.Function.load(str(path))
        self.model = SimpleNamespace(
            nq=14, nv=14,
            lowerPositionLimit=np.asarray(metadata["lower"]),
            upperPositionLimit=np.asarray(metadata["upper"]),
        )
        self._fk = functions["fk"]
        self._gravity = functions["gravity"]
        self.opti = ca.Opti()
        self.var_q = self.opti.variable(14)
        self.var_q_last = self.opti.parameter(14)
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        arguments = (self.var_q, self.param_tf_l, self.param_tf_r)
        self.opti.subject_to(self.opti.bounded(
            self.model.lowerPositionLimit, self.var_q, self.model.upperPositionLimit))
        self.opti.minimize(
            50.0 * ca.sumsqr(functions["translation"](*arguments))
            + 0.35 * ca.sumsqr(functions["rotation"](*arguments))
            + 0.02 * ca.sumsqr(self.var_q)
            + 0.1 * ca.sumsqr(self.var_q - self.var_q_last)
        )
        self.opti.solver("ipopt", {
            "expand": True, "detect_simple_bounds": True, "calc_lam_p": False,
            "print_time": False, "ipopt.sb": "yes", "ipopt.print_level": 0,
            "ipopt.max_iter": 30, "ipopt.tol": 1e-4, "ipopt.acceptable_tol": 5e-4,
            "ipopt.acceptable_iter": 5, "ipopt.warm_start_init_point": "yes",
            "ipopt.derivative_test": "none", "ipopt.jacobian_approximation": "exact",
        })
        self._reference_q = np.zeros(14)
        self.last_solution_q = np.zeros(14)
        self.has_last_solution = False
        self._queue: list = []
        self._filtered = np.zeros(14, dtype=np.float32)
        self._weights = np.asarray([0.4, 0.3, 0.2, 0.1], dtype=np.float32)
        self.last_torque = np.zeros(14)
        self.last_stats: dict = {}

    @property
    def nq(self) -> int:
        return 14

    def set_reference_q(self, reference: Sequence[float]) -> None:
        import numpy as np

        values = np.asarray(reference, dtype=np.float64).reshape(-1)
        if values.size != 14 or not np.isfinite(values).all():
            raise KinematicsError("reference needs 14 finite joint values")
        self._reference_q = values.copy()

    def _filter(self, solution):
        import numpy as np

        values = np.asarray(solution, dtype=np.float32).reshape(-1)
        if self._queue and np.array_equal(values, self._queue[-1]):
            return self._filtered.astype(np.float64)
        if len(self._queue) >= 4:
            self._queue.pop(0)
        self._queue.append(values.copy())
        if len(self._queue) < 4:
            self._filtered = self._queue[-1]
        else:
            data = np.asarray(self._queue, dtype=np.float32)
            self._filtered = np.asarray([
                np.convolve(data[:, index], self._weights, mode="valid")[-1]
                for index in range(14)
            ], dtype=np.float32)
        return self._filtered.astype(np.float64)

    def solve_reference(self, left_matrix, right_matrix, current_q=None,
                        *, use_current_q_as_initial=False):
        """Match UniRobot's q/torque outputs, including its failure fallback."""
        import numpy as np

        reference = (self.last_solution_q.copy() if current_q is None
                     else np.asarray(current_q, dtype=np.float64).reshape(-1))
        if reference.size != 14 or not np.isfinite(reference).all():
            raise KinematicsError("reference needs 14 finite joint values")
        for target in (left_matrix, right_matrix):
            if np.shape(target) != (4, 4) or not np.isfinite(target).all():
                raise KinematicsError("targets must be finite 4x4 transforms")
        initial = (reference if use_current_q_as_initial or not self.has_last_solution
                   else self.last_solution_q)
        self.opti.set_initial(self.var_q, initial)
        self.opti.set_value(self.param_tf_l, left_matrix)
        self.opti.set_value(self.param_tf_r, right_matrix)
        self.opti.set_value(self.var_q_last, reference)
        try:
            self.opti.solve()
            solution = self._filter(self.opti.value(self.var_q))
            self.last_solution_q = solution.copy()
            self.has_last_solution = True
            torque = np.asarray(self._gravity(solution), dtype=np.float64).reshape(-1)
            self.last_stats = {"success": True, "iterations": int(self.opti.stats()["iter_count"])}
        except RuntimeError as error:
            self.last_stats = {"success": False, "error": str(error)}
            self.last_torque = np.zeros(14)
            return reference, np.zeros(14)
        self.last_torque = torque.copy()
        return solution, torque

    def solve_ik(self, left: ArmPose, right: ArmPose) -> JointSolution:
        import numpy as np

        matrices = []
        for pose in (left, right):
            matrix = np.eye(4)
            matrix[:3, :3] = quaternion_to_matrix(pose.orientation_xyzw)
            matrix[:3, 3] = pose.position_m
            matrices.append(matrix)
        solution, _ = self.solve_reference(
            matrices[0], matrices[1], self._reference_q, use_current_q_as_initial=True)
        # A controller hold fallback is not a successfully planned Agent task.
        if not self.last_stats["success"]:
            raise UnreachableTargetError("UniRobot IPOPT solve failed; reference held")
        return JointSolution(tuple(float(q) for q in solution[:7]),
                             tuple(float(q) for q in solution[7:]))

    def solve_fk_matrix(self, q):
        import numpy as np

        values = np.asarray(q, dtype=np.float64).reshape(-1)
        if values.size != 14 or not np.isfinite(values).all():
            raise KinematicsError("FK needs 14 finite joint values")
        return tuple(np.asarray(pose) for pose in self._fk(values))

    def solve_fk(self, left_q, right_q) -> tuple[CartesianPose, CartesianPose]:
        if len(left_q) != 7 or len(right_q) != 7:
            raise KinematicsError("each arm needs 7 joint values")
        poses = self.solve_fk_matrix([*left_q, *right_q])
        result = [CartesianPose((float(pose[0, 3]), float(pose[1, 3]), float(pose[2, 3])),
                                matrix_to_quaternion(pose[:3, :3])) for pose in poses]
        return result[0], result[1]
