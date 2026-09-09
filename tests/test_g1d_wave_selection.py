"""Verify real IK wave selection through the public planning contract."""

import copy
import json
from pathlib import Path

import pytest
from jsonschema import ValidationError, validate

from PhyAgentOS.skill_runtime.g1d_adapter import FakeLowStateSource, make_lowstate_frame
from PhyAgentOS.skill_runtime.g1d_planner import JointLimitError, PlannerError
from PhyAgentOS.skill_runtime.g1d_runtime import G1DReadOnlyRuntime

BUNDLE = Path(__file__).parents[1] / "bundles/g1d-manipulation"


@pytest.fixture
def runtime():
    source = FakeLowStateSource()
    result = G1DReadOnlyRuntime(BUNDLE, source=source, clock=lambda: 100.)
    q = [0.] * 35
    q[15:29] = [.25, -.01, .01, 1.10, .006, .29, .009,
                .23, -.007, .078, 1.26, .021, -.065, -.011]
    source.publish(make_lowstate_frame(mode_machine=5, tick=1, positions=q), received_at=100.)
    result.poll()
    return result


@pytest.mark.parametrize("arm", ["left", "right", None])
def test_wave_raises_selected_arm_and_holds_other_from_observed_input(runtime, arm):
    state = runtime.state()
    request = copy.deepcopy(state["end_effector_poses"])
    request["gesture"] = "wave"
    if arm is not None:
        request["gesture_arm"] = arm
    schema = json.loads((BUNDLE / "tools/tools.json").read_text())["g1d.dual_arm.plan_pose"]["input_schema"]
    validate(request, schema)
    response = runtime.plan_pose(request)
    plan = runtime.planner.get_plan(response["plan_id"])
    assert response["binding"]["skill_version"] == runtime.skill_version == state["skill_version"]
    selected = arm or "right"
    held = slice(7, 14) if selected == "left" else slice(0, 7)
    wrist = 6 if selected == "left" else 13
    points = plan.joint_path.points
    assert points[0] == points[-1] == plan.start_q
    assert all(p[held] == plan.start_q[held] for p in points)
    assert max(p[wrist] for p in points[1:-1]) - min(p[wrist] for p in points[1:-1]) == pytest.approx(.7)
    poses = runtime.kinematics.solve_fk(points[1][:7], points[1][7:])
    raised = poses[0 if selected == "left" else 1]
    assert raised.position_m[2] > 1.0
    assert raised.position_m[2] - state["end_effector_poses"][selected]["position_m"][2] > .3


@pytest.mark.parametrize("selection", ["both", "LEFT", "front", ""])
def test_unknown_arm_is_rejected(runtime, selection):
    request = {**runtime.state()["end_effector_poses"], "gesture": "wave", "gesture_arm": selection}
    with pytest.raises(PlannerError, match="left or right"):
        runtime.plan_pose(request)


def test_arm_selection_requires_wave(runtime):
    request = {**runtime.state()["end_effector_poses"], "gesture_arm": "left"}
    schema = json.loads((BUNDLE / "tools/tools.json").read_text())["g1d.dual_arm.plan_pose"]["input_schema"]
    with pytest.raises(ValidationError):
        validate(request, schema)
    with pytest.raises(PlannerError, match="requires gesture"):
        runtime.plan_pose(request)


def test_left_wrist_limit_violation_cannot_produce_wave(runtime):
    request = runtime.state()["end_effector_poses"]
    runtime.kinematics.set_reference_q(runtime._positions[15:29])
    base = runtime.planner.plan_pose(**request, current_q=runtime._positions)
    limits = list(runtime.planner._joint_limits)
    centre = base.joint_solution.left_q[6]
    limits[6] = (centre - .1, centre + .1)
    runtime.planner._joint_limits = tuple(limits)
    with pytest.raises(JointLimitError):
        runtime.planner.add_wave(base, arm="left")
