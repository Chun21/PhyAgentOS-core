from pathlib import Path

import pytest

from PhyAgentOS.skill_runtime.g1d_adapter import FakeLowStateSource, make_lowstate_frame
from PhyAgentOS.skill_runtime.g1d_dex1 import Dex1CommandSample, Dex1Integration, Dex1NotReadyError
from PhyAgentOS.skill_runtime.g1d_execution_runtime import G1DExecutionRuntime
from PhyAgentOS.skill_runtime.g1d_executor import RecordingSink


@pytest.mark.parametrize("openings", [{"left": .8}, {"right": .2}, {"left": .7, "right": .4}])
def test_native_gripper_plan_and_execute_hold_both_arms_exactly(tmp_path, monkeypatch, openings):
    now = [100.]
    source, sink = FakeLowStateSource(), RecordingSink()
    dex = Dex1Integration(clock=lambda: now[0])
    runtime = G1DExecutionRuntime(Path(__file__).parents[1] / "bundles/g1d-manipulation",
        source=source, sink=sink, journal_path=tmp_path / "executor.sqlite", approved_mode=5,
        gains=[(20, 1)] * 35, modes=[1] * 35, require_ownership=lambda: None,
        clock=lambda: now[0], dex1=dex)
    q = [0.] * 35
    q[15:29] = [.25, -.01, .01, 1.10, .006, .29, .009, .23, -.007, .078, 1.26, .021, -.065, -.011]
    def feedback(tick):
        source.publish(make_lowstate_frame(mode_machine=5, tick=tick, positions=q), received_at=now[0])
        runtime.poll()
    def no_ik(*args):
        pytest.fail("gripper-only actions must not use IK")
    monkeypatch.setattr(runtime.kinematics, "solve_ik", no_ik)
    feedback(1)
    try:
        with pytest.raises(Dex1NotReadyError):
            runtime.plan_gripper(openings)
        for side in openings:
            dex.publish(side, opening=.5, received_at=now[0])
        planned = runtime.plan_gripper(openings)
        plan = runtime.planner.get_plan(planned["plan_id"])
        assert (*plan.joint_solution.left_q, *plan.joint_solution.right_q) == tuple(q[15:29])
        invocation = runtime.executor.execute_pose(plan_id=plan.plan_id, caller_id="test-gripper")
        for tick in range(2, 3000):
            now[0] += .002
            feedback(tick)
            for side, target in openings.items():
                dex.publish(side, opening=target, received_at=now[0])
            runtime.executor.tick()
            if invocation.status.is_terminal:
                break
        assert invocation.status.value == "succeeded", invocation
        arms = [s for s in sink.samples if not isinstance(s, Dex1CommandSample)]
        assert arms and all(tuple(m.q for m in s.frame.motor_cmd[15:29]) == tuple(q[15:29]) for s in arms)
        fingers = [s for s in sink.samples if isinstance(s, Dex1CommandSample)]
        assert {s.command.side for s in fingers} == set(openings)
        assert all(s.command.opening == openings[s.command.side] for s in fingers)
    finally:
        runtime.executor.close()
