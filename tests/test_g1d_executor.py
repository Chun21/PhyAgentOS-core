from __future__ import annotations

from pathlib import Path

import pytest

from PhyAgentOS.skill_runtime.g1d_adapter import (
    TOTAL_MOTOR_SLOTS,
    G1DAdapter,
    StaleStateError,
    StateUnavailableError,
    make_lowstate_frame,
)
from PhyAgentOS.skill_runtime.g1d_executor import (
    ActiveActionError,
    BindingMismatchError,
)
from PhyAgentOS.skill_runtime.g1d_planner import (
    FixtureKinematics,
    G1DPlanner,
    PlanExpiredError,
    digest_json,
    load_kinematics_profile,
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

ARM_START = [0.0] * 15 + [0.1] * 7 + [-0.1] * 7 + [0.0] * 6


class Clock:
    def __init__(self, value: float = 1000.0) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


class Harness:
    """Robot-free seam: fake lowstate, fixture kinematics, controllable clock."""

    def __init__(self, *, runtime_instance_id: str = "runtime-test") -> None:
        self.clock = Clock()
        self.adapter = G1DAdapter(clock=self.clock, max_state_age_s=0.1, approved_mode_machine=7)
        profile = load_kinematics_profile(KINEMATICS)
        self.planner = G1DPlanner(
            kinematics=FixtureKinematics(
                fixtures=[(dict(HOME_LEFT), dict(HOME_RIGHT), HOME_LEFT_Q, HOME_RIGHT_Q)]
            ),
            clock=self.clock,
            skill_version="0.1.0",
            runtime_instance_id=runtime_instance_id,
            profile_digest=digest_json(profile),
        )
        self.state_tick = 0
        self.executor = None  # set by make_executor once implemented

    def publish_state(
        self, *, positions: list[float] | None = None, mode_machine: int = 7, safety_ok: bool = True
    ) -> None:
        self.state_tick += 1
        frame = make_lowstate_frame(
            mode_machine=mode_machine,
            tick=self.state_tick,
            positions=positions if positions is not None else ARM_START,
            mode=[1] * 35,
            safety_ok=safety_ok,
        )
        self.adapter.ingest(frame, received_at=self.clock())

    def make_plan(self):
        return self.planner.plan_pose(left=HOME_LEFT, right=HOME_RIGHT, current_q=ARM_START)


def make_executor(harness: Harness, **binding_overrides):
    from PhyAgentOS.skill_runtime.g1d_executor import G1DExecutor, RecordingSink

    binding = {
        "skill_version": "0.1.0",
        "runtime_instance_id": "runtime-test",
        "profile_digest": digest_json(load_kinematics_profile(KINEMATICS)),
    }
    binding.update(binding_overrides)
    sink = RecordingSink()
    harness.executor = G1DExecutor(
        adapter=harness.adapter,
        planner=harness.planner,
        clock=harness.clock,
        sink=sink,
        **binding,
    )
    harness.sink = sink
    return harness.executor


def test_execute_rejects_unknown_plan() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)

    with pytest.raises(PlanExpiredError):
        executor.execute_pose(plan_id="plan-does-not-exist", caller_id="agent-1")


def test_execute_rejects_expired_plan() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()

    harness.clock.advance(5.1)  # past the five-second plan TTL
    harness.publish_state()

    with pytest.raises(PlanExpiredError):
        executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")


def test_execute_rejects_binding_mismatch() -> None:
    # A runtime restart (new instance id) must reject plans bound to the old
    # runtime identity even though the planner still knows the plan.
    harness = Harness()
    harness.publish_state()
    plan = harness.make_plan()
    executor = make_executor(harness, runtime_instance_id="runtime-restarted")

    with pytest.raises(BindingMismatchError, match="binding"):
        executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")


def test_execute_admits_current_plan_as_pending_invocation() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()

    invocation = executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")

    assert invocation.invocation_id
    assert invocation.attempt_id
    assert invocation.plan_id == plan.plan_id
    assert invocation.caller_id == "agent-1"
    assert invocation.status.value == "pending"
    assert invocation.deadline_at > harness.clock()


def test_duplicate_caller_plan_retry_reconciles_to_one_invocation() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()

    first = executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    again = executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")

    assert again.invocation_id == first.invocation_id
    assert again.attempt_id == first.attempt_id


def test_second_distinct_action_is_rejected_while_one_is_active() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()

    executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")

    with pytest.raises(ActiveActionError, match="active"):
        executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-2")


def test_execute_requires_fresh_ready_state() -> None:
    harness = Harness()
    executor = make_executor(harness)
    plan = harness.make_plan()

    with pytest.raises(StateUnavailableError):
        executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")

    harness.publish_state()
    harness.clock.advance(0.101)
    with pytest.raises(StaleStateError):
        executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")


# ---------------------------------------------------------------------------
# Streaming, watchdog, lifecycle, and stop (issue #13 slice two)
# ---------------------------------------------------------------------------


def run_to_completion(harness: Harness, *, publish_every_tick: bool = True) -> None:
    """Drive the executor clock tick by tick until the invocation terminates."""

    executor = harness.executor
    for _ in range(200_000):
        if executor.get_active_status().is_terminal:
            return
        if publish_every_tick:
            positions = (
                [m.q for m in harness.sink.samples[-1].frame.motor_cmd]
                if harness.sink.samples
                else ARM_START
            )
            harness.publish_state(positions=positions)
        harness.clock.advance(0.002)
        executor.tick()
    raise AssertionError("invocation never reached a terminal status")


def test_hold_precedes_minimum_one_second_synchronized_quintic_motion() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()
    executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")

    run_to_completion(harness)

    samples = harness.sink.samples
    # Initial hold frames replay the current state before any motion.
    assert samples[0].phase.value == "hold"
    hold_positions = [
        sample.frame.motor_cmd[15].q for sample in samples if sample.phase.value == "hold"
    ]
    assert hold_positions[0] == ARM_START[15]

    motion = [sample for sample in samples if sample.phase.value == "motion"]
    motion_span = motion[-1].t_s - motion[0].t_s
    assert motion_span >= 1.0  # one-second minimum duration

    # Both arms follow one shared quintic parameterization: slot pairs at the
    # same sample share the same progress fraction, so left and right deltas
    # stay proportional to their total travel throughout the motion.
    start_left = ARM_START[15]
    target_left = HOME_LEFT_Q[0]
    start_right = ARM_START[22]
    target_right = HOME_RIGHT_Q[0]
    for sample in motion[1:-1]:
        progress_left = (sample.frame.motor_cmd[15].q - start_left) / (target_left - start_left)
        progress_right = (sample.frame.motor_cmd[22].q - start_right) / (target_right - start_right)
        assert progress_left == pytest.approx(progress_right, abs=1e-9)

    # Quintic boundary conditions: zero velocity at start and end means the
    # first and last motion samples barely move.
    first_step = abs(motion[1].frame.motor_cmd[15].q - motion[0].frame.motor_cmd[15].q)
    mid_index = len(motion) // 2
    mid_step = abs(
        motion[mid_index + 1].frame.motor_cmd[15].q - motion[mid_index].frame.motor_cmd[15].q
    )
    assert first_step < mid_step / 10

    # Final frames hold the planned joint targets.
    final = samples[-1].frame
    assert [final.motor_cmd[slot].q for slot in range(15, 22)] == pytest.approx(HOME_LEFT_Q)
    assert [final.motor_cmd[slot].q for slot in range(22, 29)] == pytest.approx(HOME_RIGHT_Q)
    # Slots 0-14 are preserved at their current-state values.
    assert [final.motor_cmd[slot].q for slot in range(15)] == pytest.approx(ARM_START[:15])


def test_stream_frames_are_complete_crc_valid_sequenced_and_monotonic_at_2ms() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()
    executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")

    run_to_completion(harness)

    samples = harness.sink.samples
    assert len(samples) > 500
    for index, sample in enumerate(samples):
        frame = sample.frame
        assert len(frame.motor_cmd) == TOTAL_MOTOR_SLOTS
        assert frame.crc == frame.calculated_crc(), f"CRC invalid at sample {index}"
        assert frame.mode_machine == 7
        assert sample.seq == index
    timestamps = [sample.t_s for sample in samples]
    assert all(b > a for a, b in zip(timestamps, timestamps[1:]))
    assert all(
        (b - a) == pytest.approx(0.002, abs=1e-9) for a, b in zip(timestamps, timestamps[1:])
    )
    # Executor reports the terminal success outcome.
    assert executor.get_active_status().value == "succeeded"


def test_ten_missed_cycles_trigger_watchdog_stop() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()
    executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    harness.publish_state()
    harness.clock.advance(0.002)
    executor.tick()
    assert executor.get_active_status().value == "running"

    # The writer loop stalls for more than ten 2 ms cycles.
    harness.publish_state()
    harness.clock.advance(10 * 0.002 + 0.001)
    executor.tick()

    status = executor.get_active_status()
    assert status.is_terminal
    assert status.value == "unknown"
    invocation = executor.active_invocation()
    assert invocation.stop_reason == "watchdog_missed_cycles"


def test_state_age_over_100ms_triggers_watchdog_stop() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()
    executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")

    harness.publish_state()
    harness.clock.advance(0.002)
    executor.tick()

    # State goes stale (>100 ms) but the writer keeps its 2 ms cadence.
    for _ in range(60):
        harness.clock.advance(0.002)
        executor.tick()

    status = executor.get_active_status()
    assert status.is_terminal
    assert status.value == "unknown"
    assert executor.active_invocation().stop_reason == "stale_state"


def test_lifecycle_success_and_terminal_statuses() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()
    invocation = executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    assert invocation.status.value == "pending"

    harness.publish_state()
    harness.clock.advance(0.002)
    executor.tick()
    assert executor.get_active_status().value == "running"

    run_to_completion(harness)
    assert executor.get_active_status().value == "succeeded"
    assert executor.active_invocation().frames_emitted == len(harness.sink.samples)


def test_operator_stop_during_motion_ends_stopped_and_pre_effect_cancelled() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()

    # Pre-effect stop: admission only, no tick emitted yet.
    invocation = executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    assert executor.stop(invocation_id=invocation.invocation_id) == "accepted"
    harness.publish_state()
    harness.clock.advance(0.002)
    executor.tick()
    assert executor.get_active_status().value == "cancelled"

    # Stop during motion ends stopped after the stop margin hold.
    harness2 = Harness()
    harness2.publish_state()
    executor2 = make_executor(harness2)
    plan2 = harness2.make_plan()
    invocation2 = executor2.execute_pose(plan_id=plan2.plan_id, caller_id="agent-1")
    for _ in range(10):
        harness2.publish_state()
        harness2.clock.advance(0.002)
        executor2.tick()
    assert executor2.get_active_status().value == "running"
    assert executor2.stop(invocation_id=invocation2.invocation_id) == "accepted"
    assert executor2.get_active_status().value == "stopping"
    for _ in range(500):
        harness2.publish_state()
        harness2.clock.advance(0.002)
        executor2.tick()
        if executor2.get_active_status().is_terminal:
            break
    assert executor2.get_active_status().value == "stopped"
    assert executor2.active_invocation().stop_reason == "operator_stop"


def test_deadline_exceeded_ends_terminal_after_stop_handling() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()
    executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1", operation_deadline_s=1.0)

    for _ in range(1000):
        harness.publish_state()
        harness.clock.advance(0.002)
        executor.tick()
        if executor.get_active_status().is_terminal:
            break

    assert executor.get_active_status().value == "deadline_exceeded"


def test_safety_fault_ends_failed_and_stop_is_idempotent() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()
    invocation = executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")

    for _ in range(10):
        harness.publish_state()
        harness.clock.advance(0.002)
        executor.tick()

    harness.publish_state(safety_ok=False)
    harness.clock.advance(0.002)
    executor.tick()
    assert executor.get_active_status().value == "unknown"

    # Unknown is not confirmation that the robot stopped.
    assert executor.stop(invocation_id=invocation.invocation_id) == "unknown"
    assert executor.stop() == "unknown"
    assert executor.stop(invocation_id="inv-nonsense") == "unknown"


def test_unknown_outcome_is_explicit_and_never_retried_implicitly() -> None:
    harness = Harness()
    harness.publish_state()
    executor = make_executor(harness)
    plan = harness.make_plan()
    invocation = executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    for _ in range(10):
        harness.publish_state()
        harness.clock.advance(0.002)
        executor.tick()

    # Transport loss: the runtime marks the invocation unknown.
    executor.mark_unknown(invocation.invocation_id, reason="transport_loss")

    assert executor.get_invocation(invocation.invocation_id).status.value == "unknown"
    # A new execute for the same caller/plan reconciles to the same invocation
    # instead of silently starting a second physical Action.
    retried = executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    assert retried.invocation_id == invocation.invocation_id


def test_delayed_tick_does_not_burst_commands_or_backdate_evidence():
    h = Harness()
    h.publish_state()
    ex = make_executor(h)
    ex.execute_pose(plan_id=h.make_plan().plan_id, caller_id="a")
    ex.tick()
    h.clock.advance(0.008)
    h.publish_state()
    ex.tick()
    assert len(h.sink.samples) == 2
    assert h.sink.samples[-1].t_s == h.clock()


def test_restart_reconciles_durable_identity_without_replaying(tmp_path):
    h = Harness()
    h.publish_state()
    ex = make_executor(h, journal_path=tmp_path / "actions.sqlite")
    plan = h.make_plan()
    first = ex.execute_pose(plan_id=plan.plan_id, caller_id="a")
    ex.tick()
    ex.close()
    restarted = make_executor(h, journal_path=tmp_path / "actions.sqlite")
    record = restarted.execute_pose(plan_id=plan.plan_id, caller_id="a")
    assert record.invocation_id == first.invocation_id
    assert record.status.value == "unknown"
    restarted.tick()
    assert h.sink.samples == []
    restarted.close()


def test_stale_feedback_is_unknown_not_confirmed_stop():
    h = Harness()
    h.publish_state()
    ex = make_executor(h)
    ex.execute_pose(plan_id=h.make_plan().plan_id, caller_id="a")
    ex.tick()
    h.clock.advance(0.101)
    ex.tick()
    assert ex.active_invocation().status.value == "unknown"


def test_command_completion_without_target_feedback_is_not_success():
    h = Harness()
    h.publish_state()
    ex = make_executor(h)
    ex.execute_pose(plan_id=h.make_plan().plan_id, caller_id="a", operation_deadline_s=8)
    for _ in range(4100):
        h.publish_state()
        h.clock.advance(0.002)
        ex.tick()
        if ex.active_invocation().status.is_terminal:
            break
    assert ex.active_invocation().status.value != "succeeded"


def test_stop_without_matching_feedback_is_unknown():
    h = Harness()
    h.publish_state()
    ex = make_executor(h)
    ex.execute_pose(plan_id=h.make_plan().plan_id, caller_id="a")
    for _ in range(700):
        h.publish_state()
        h.clock.advance(0.002)
        ex.tick()
    ex.stop()
    for _ in range(2000):
        h.publish_state()
        h.clock.advance(0.002)
        ex.tick()
        if ex.active_invocation().status.is_terminal:
            break
    assert ex.active_invocation().status.value == "unknown"


def test_stop_decelerates_and_requires_observed_hold():
    h = Harness()
    h.publish_state()
    ex = make_executor(h)
    ex.execute_pose(plan_id=h.make_plan().plan_id, caller_id="a")
    for _ in range(700):
        h.publish_state()
        h.clock.advance(0.002)
        ex.tick()
    before = h.sink.samples[-1]
    assert abs(before.frame.motor_cmd[21].dq) > 0.1
    assert ex.stop() == "accepted"
    run_to_completion(h)
    assert ex.active_invocation().status.value == "stopped"
    after = [s for s in h.sink.samples if s.t_s > before.t_s]
    assert after[0].frame.motor_cmd[21].dq == pytest.approx(before.frame.motor_cmd[21].dq, abs=0.01)
    assert after[-1].frame.motor_cmd[21].dq == 0
    speeds = [s.frame.motor_cmd[21].dq for s in after]
    accelerations = [(b - a) / 0.002 for a, b in zip(speeds, speeds[1:])]
    assert max(map(abs, speeds)) <= 0.5
    assert max(map(abs, accelerations)) <= 2
    assert max(abs(b - a) / 0.002 for a, b in zip(accelerations, accelerations[1:])) <= 10.01


def test_second_executor_cannot_reconcile_a_live_journal(tmp_path):
    h = Harness()
    h.publish_state()
    first = make_executor(h, journal_path=tmp_path / "actions.sqlite")
    with pytest.raises((OSError, RuntimeError)):
        make_executor(h, journal_path=tmp_path / "actions.sqlite")
    first.close()


def test_unknown_outcome_closes_readiness_and_stop_does_not_claim_stopped():
    h = Harness()
    h.publish_state()
    ex = make_executor(h)
    record = ex.execute_pose(plan_id=h.make_plan().plan_id, caller_id="a")
    ex.mark_unknown(record.invocation_id, reason="transport_loss")
    assert not ex.query_state()["action_ready"]
    assert ex.stop() == "unknown"
    assert ex.stop(invocation_id=record.invocation_id) == "unknown"


def test_completed_invocation_retains_before_during_after_evidence(tmp_path):
    h = Harness()
    h.publish_state()
    ex = make_executor(h, journal_path=tmp_path / "actions.sqlite")
    record = ex.execute_pose(plan_id=h.make_plan().plan_id, caller_id="a")
    run_to_completion(h)
    evidence = ex.evidence(record.invocation_id)
    assert [item["phase"] for item in evidence] == ["before", "during", "after"]
    assert evidence[1]["payload"]["frames"][-1]["seq"] + 1 == record.frames_emitted
    assert evidence[2]["payload"]["status"] == "succeeded"
    from PhyAgentOS.verification.g1d_execution import verify_execution

    assert verify_execution(evidence).verdict == "success"
    import copy

    tampered = copy.deepcopy(evidence)
    tampered[1]["payload"]["observations"] = []
    assert verify_execution(tampered).verdict == "inconclusive"
    relabelled = copy.deepcopy(evidence)
    for item in relabelled:
        item["invocation_id"] = "another-operation"
    assert verify_execution(relabelled).verdict == "inconclusive"
    ex.close()
    restored = make_executor(h, journal_path=tmp_path / "actions.sqlite")
    assert restored.evidence(record.invocation_id) == evidence
    restored.close()


def test_writer_failure_is_unknown_and_cannot_resume():
    h = Harness()
    h.publish_state()
    ex = make_executor(h)
    record = ex.execute_pose(plan_id=h.make_plan().plan_id, caller_id="a")

    def fail(sample):
        raise OSError("DDS transport lost")

    h.sink.write = fail
    with pytest.raises(OSError):
        ex.tick()
    assert record.status.value == "unknown"
    ex.tick()
    assert ex.stop() == "unknown"


def test_real_scheduler_streams_without_gateway_polling():
    import threading
    import time

    from PhyAgentOS.skill_runtime.g1d_control_loop import G1DControlLoop
    from PhyAgentOS.skill_runtime.g1d_executor import G1DExecutor

    h = Harness()
    h.clock = time.monotonic
    h.adapter = G1DAdapter(clock=h.clock, approved_mode_machine=7)
    # Use the same binding as the existing planner, whose clock starts at 1000.
    # Admission must use a plan with the real monotonic clock, too.
    h.planner = G1DPlanner(
        kinematics=FixtureKinematics(
            fixtures=[(dict(HOME_LEFT), dict(HOME_RIGHT), HOME_LEFT_Q, HOME_RIGHT_Q)]
        ),
        clock=h.clock,
        skill_version="test",
        runtime_instance_id="live",
        profile_digest="profile",
    )
    event = threading.Event()
    samples = []

    class Sink:
        def write(self, sample):
            samples.append(sample)
            if len(samples) >= 10:
                event.set()

    h.publish_state()
    executor = G1DExecutor(
        adapter=h.adapter,
        planner=h.planner,
        clock=h.clock,
        sink=Sink(),
        skill_version="test",
        runtime_instance_id="live",
        profile_digest="profile",
    )
    executor.execute_pose(plan_id=h.make_plan().plan_id, caller_id="a")
    loop = G1DControlLoop(executor, h.publish_state)
    loop.start()
    try:
        assert event.wait(2)
        assert loop.error is None
        assert all(b.t_s > a.t_s for a, b in zip(samples, samples[1:]))
    finally:
        loop.close()
