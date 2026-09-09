"""Cross-toolchain parity against outputs recorded from unmodified UniRobot."""

import json
from pathlib import Path

import numpy as np
import pytest

from PhyAgentOS.skill_runtime.g1d_kinematics_unirobot import UniRobotKinematics


def test_reference_sequence_matches_angles_torque_and_fk():
    cases = json.loads((Path(__file__).parent / "fixtures/g1d_unirobot_ik.json").read_text())
    for case in cases:
        if case["reset"]:
            solver = UniRobotKinematics()
        q, torque = solver.solve_reference(
            np.asarray(case["left"]), np.asarray(case["right"]), case["reference"],
            use_current_q_as_initial=case["current_initial"],
        )
        assert solver.last_stats["success"]
        np.testing.assert_allclose(q, case["q"], atol=2e-6, rtol=0)
        np.testing.assert_allclose(torque, case["torque"], atol=2e-5, rtol=0)
        np.testing.assert_allclose(solver.solve_fk_matrix(q), case["fk"], atol=2e-6, rtol=0)


def test_graph_integrity_checked(tmp_path):
    import shutil
    from PhyAgentOS.skill_runtime.g1d_kinematics_unirobot import model_directory
    from PhyAgentOS.skill_runtime.g1d_planner import KinematicsError

    shutil.copytree(model_directory(), tmp_path / "model")
    (tmp_path / "model/fk.casadi").write_text("corrupt")
    with pytest.raises(KinematicsError, match="hash"):
        UniRobotKinematics(directory=tmp_path / "model")


def test_failed_solve_holds_reference_but_agent_plan_rejects(monkeypatch):
    from PhyAgentOS.skill_runtime.g1d_planner import ArmPose, UnreachableTargetError

    solver = UniRobotKinematics()
    reference = np.full(14, .1)
    poses = solver.solve_fk_matrix(reference)

    def fail():
        raise RuntimeError("forced IPOPT failure")

    monkeypatch.setattr(solver.opti, "solve", fail)
    q, tau = solver.solve_reference(*poses, reference)
    np.testing.assert_array_equal(q, reference)
    np.testing.assert_array_equal(tau, np.zeros(14))
    assert not solver.has_last_solution and not solver._queue
    solver.set_reference_q(reference)
    pose = ArmPose(frame_id="unirobot_g1d_fixed_base", position_m=(0, 0, 0),
                   orientation_xyzw=(0, 0, 0, 1))
    with pytest.raises(UnreachableTargetError):
        solver.solve_ik(pose, pose)
