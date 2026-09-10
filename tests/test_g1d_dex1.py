from __future__ import annotations

import json
from pathlib import Path

import pytest

from PhyAgentOS.skill_runtime.g1d_adapter import G1DAdapter, make_lowstate_frame
from PhyAgentOS.skill_runtime.g1d_dex1 import (
    Dex1CommandSample,
    Dex1Integration,
    Dex1Status,
    FakeDex1StateSource,
    InvalidOpeningError,
)
from PhyAgentOS.skill_runtime.g1d_executor import Dex1GateError, G1DExecutor, RecordingSink
from PhyAgentOS.skill_runtime.g1d_planner import (
    FixtureKinematics,
    G1DPlanner,
    digest_json,
    load_kinematics_profile,
)

BUNDLE = Path(__file__).parents[1] / "bundles" / "g1d-manipulation"
KINEMATICS = BUNDLE / "profiles" / "real-g1d" / "kinematics.json"
TOOLS = BUNDLE / "tools" / "tools.json"

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


def _dex1_target(opening: float) -> dict:
    return {"opening": opening}


class Harness:
    def __init__(self) -> None:
        self.clock = Clock()
        self.adapter = G1DAdapter(
            clock=self.clock, max_state_age_s=0.1, approved_mode_machine=7
        )
        profile = load_kinematics_profile(KINEMATICS)
        self.planner = G1DPlanner(
            kinematics=FixtureKinematics(
                fixtures=[
                    # The opening participates in the target digest, so each
                    # requested-opening variant needs its own recorded fixture.
                    (dict(HOME_LEFT), dict(HOME_RIGHT), HOME_LEFT_Q, HOME_RIGHT_Q),
                    (
                        {**HOME_LEFT, "dex1": {"opening": 0.6}},
                        dict(HOME_RIGHT),
                        HOME_LEFT_Q,
                        HOME_RIGHT_Q,
                    ),
                    (
                        dict(HOME_LEFT),
                        {**HOME_RIGHT, "dex1": {"opening": 0.3}},
                        HOME_LEFT_Q,
                        HOME_RIGHT_Q,
                    ),
                    (
                        {**HOME_LEFT, "dex1": {"opening": 0.6}},
                        {**HOME_RIGHT, "dex1": {"opening": 0.2}},
                        HOME_LEFT_Q,
                        HOME_RIGHT_Q,
                    ),
                ]
            ),
            clock=self.clock,
            skill_version="0.1.0",
            runtime_instance_id="runtime-test",
            profile_digest=digest_json(profile),
        )
        self.dex1 = Dex1Integration(clock=self.clock)
        self.sink = RecordingSink()
        self.executor = G1DExecutor(
            adapter=self.adapter,
            planner=self.planner,
            clock=self.clock,
            sink=self.sink,
            skill_version="0.1.0",
            runtime_instance_id="runtime-test",
            profile_digest=digest_json(profile),
            dex1=self.dex1,
        )

    def publish_arm_state(self) -> None:
        self.adapter.ingest(
            make_lowstate_frame(
                mode_machine=7, tick=42, positions=ARM_START, mode=[1] * 35
            ),
            received_at=self.clock(),
        )

    def publish_dex1(self, side: str, opening: float) -> None:
        self.dex1.publish(side, opening=opening, received_at=self.clock())

    def make_plan(self, *, left_dex1: dict | None = None, right_dex1: dict | None = None):
        left = dict(HOME_LEFT)
        right = dict(HOME_RIGHT)
        if left_dex1 is not None:
            left["dex1"] = left_dex1
        if right_dex1 is not None:
            right["dex1"] = right_dex1
        return self.planner.plan_pose(left=left, right=right, current_q=ARM_START)


# ---------------------------------------------------------------------------
# Opening validation
# ---------------------------------------------------------------------------


def test_opening_commands_must_be_finite_and_within_unit_range() -> None:
    dex1 = Dex1Integration(clock=Clock())

    for bad in (1.5, -0.1, float("nan"), float("inf")):
        with pytest.raises(InvalidOpeningError):
            dex1.command("left", bad)

    for good in (0.0, 0.5, 1.0):
        command = dex1.command("left", good)
        assert command.topic == "rt/dex1/left/cmd"
        assert command.opening == good
    right = dex1.command("right", 0.25)
    assert right.topic == "rt/dex1/right/cmd"


# ---------------------------------------------------------------------------
# Readiness: healthy / absent / stale (timeout)
# ---------------------------------------------------------------------------


def test_readiness_distinguishes_healthy_absent_and_stale() -> None:
    clock = Clock()
    dex1 = Dex1Integration(clock=clock)

    # Absent: the external service has never published this side.
    absent = dex1.readiness("left")
    assert absent.status is Dex1Status.ABSENT
    assert absent.ok is False
    assert absent.timed_out is False

    # Healthy: fresh state within the freshness window.
    dex1.publish("left", opening=0.4, received_at=clock())
    healthy = dex1.readiness("left")
    assert healthy.status is Dex1Status.HEALTHY
    assert healthy.ok is True
    assert healthy.opening == 0.4
    assert healthy.age_ms == 0.0

    # Stale / timed out: state exists but is older than 100 ms.
    clock.advance(0.101)
    stale = dex1.readiness("left")
    assert stale.status is Dex1Status.STALE
    assert stale.ok is False
    assert stale.timed_out is True
    assert stale.age_ms == pytest.approx(101.0)


# ---------------------------------------------------------------------------
# Plan / execute / state integration
# ---------------------------------------------------------------------------


def test_missing_opening_is_non_blocking_and_requested_opening_is_carried_by_plan() -> None:
    harness = Harness()
    harness.publish_arm_state()

    plain = harness.make_plan()
    assert plain.dex1_left_opening is None
    assert plain.dex1_right_opening is None
    # No Dex1 state published at all: execution without openings still admits.
    invocation = harness.executor.execute_pose(plan_id=plain.plan_id, caller_id="agent-1")
    assert invocation.status.value == "pending"

    requested = harness.make_plan(
        left_dex1=_dex1_target(0.6), right_dex1=_dex1_target(0.2)
    )
    assert requested.dex1_left_opening == 0.6
    assert requested.dex1_right_opening == 0.2


def test_requested_execution_is_gated_on_fresh_dex1_state() -> None:
    harness = Harness()
    harness.publish_arm_state()
    plan = harness.make_plan(left_dex1=_dex1_target(0.6))

    # Left requested but absent -> gated.
    with pytest.raises(Dex1GateError, match="left"):
        harness.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")

    # Healthy -> admitted.
    harness.publish_dex1("left", 0.4)
    invocation = harness.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    assert invocation.status.value == "pending"

    # Right requested but stale -> gated with a timeout indication.
    stale_harness = Harness()
    stale_harness.publish_arm_state()
    plan2 = stale_harness.make_plan(right_dex1=_dex1_target(0.3))
    stale_harness.publish_dex1("right", 0.3)
    stale_harness.clock.advance(0.15)
    stale_harness.publish_arm_state()  # arm state stays fresh; only Dex1 times out
    with pytest.raises(Dex1GateError, match="right"):
        stale_harness.executor.execute_pose(plan_id=plan2.plan_id, caller_id="agent-1")


def test_state_query_reports_dex1_readiness_per_side() -> None:
    harness = Harness()
    harness.publish_arm_state()
    harness.publish_dex1("left", 0.4)

    state = harness.executor.query_state()

    assert state["runtime_ready"] is True
    assert state["action_ready"] is True
    assert state["state_age_ms"] == 0.0
    assert state["safety_gate"] == "ready"
    assert state["dex1"]["left"]["status"] == "healthy"
    assert state["dex1"]["left"]["opening"] == 0.4
    assert state["dex1"]["right"]["status"] == "absent"

    # A runtime without any ingested arm state reports not ready instead of
    # crashing: state must be honest, not optimistic.
    empty = Harness()
    empty_state = empty.executor.query_state()
    assert empty_state["runtime_ready"] is False
    assert empty_state["action_ready"] is False


# ---------------------------------------------------------------------------
# External lifecycle boundary
# ---------------------------------------------------------------------------


def test_dex1_service_lifecycle_stays_outside_normal_tools() -> None:
    tools = json.loads(TOOLS.read_text())

    assert set(tools) == {
        "g1d.dual_arm.plan_pose",
        "g1d.dual_arm.execute_pose",
        "g1d.dual_arm.state",
        "g1d.dual_arm.stop",
        "g1d.dual_arm.plan_gripper",
        "g1d.dual_arm.camera_state",
        "g1d.dual_arm.camera_observe",
    }
    operations = {spec["operation"] for spec in tools.values()}
    assert operations == {"plan_pose", "execute_pose", "state", "stop", "plan_gripper", "camera_state", "camera_observe"}
    # No Tool installs, calibrates, restarts, or stops the external Dex1
    # service; the only stop targets the active arm Action.
    blob = " ".join(operations).lower()
    for forbidden in ("install", "calibrat", "restart"):
        assert forbidden not in blob
    # The only stop Tool stops the active arm Action, never the Dex1 service.
    assert "dex1" not in tools["g1d.dual_arm.stop"]["description"].lower()


def test_fake_dex1_source_feeds_the_integration_without_a_robot() -> None:
    clock = Clock()
    source = FakeDex1StateSource()
    dex1 = Dex1Integration(clock=clock, sources={"left": source})

    assert dex1.readiness("left").status is Dex1Status.ABSENT
    source.publish(opening=0.7, received_at=clock())
    assert dex1.readiness("left").status is Dex1Status.HEALTHY
    assert dex1.readiness("left").opening == 0.7
    # A side with no source wired stays absent and non-blocking when unused.
    assert dex1.readiness("right").status is Dex1Status.ABSENT


def test_requested_opening_is_commanded_during_execution() -> None:
    harness = Harness()
    harness.publish_arm_state()
    harness.publish_dex1("left", 0.4)
    harness.publish_dex1("right", 0.5)
    plan = harness.make_plan(left_dex1=_dex1_target(0.6))

    harness.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    for _ in range(30):
        harness.publish_arm_state()
        harness.publish_dex1("left", 0.4)
        harness.publish_dex1("right", 0.5)
        harness.clock.advance(0.002)
        harness.executor.tick()

    dex1_samples = [
        sample for sample in harness.sink.samples if isinstance(sample, Dex1CommandSample)
    ]
    assert dex1_samples
    assert all(sample.command.side == "left" for sample in dex1_samples)
    assert all(sample.command.opening == 0.6 for sample in dex1_samples)
    assert all(sample.command.topic == "rt/dex1/left/cmd" for sample in dex1_samples)


def test_dex1_stream_going_stale_mid_execution_stops_the_action() -> None:
    harness = Harness()
    harness.publish_arm_state()
    harness.publish_dex1("left", 0.4)
    plan = harness.make_plan(left_dex1=_dex1_target(0.6))
    harness.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")

    # Arm state stays fresh; the Dex1 stream stops updating and times out.
    for _ in range(100):
        harness.publish_arm_state()
        harness.clock.advance(0.002)
        harness.executor.tick()
        if harness.executor.get_active_status() is not None and harness.executor.get_active_status().is_terminal:
            break

    status = harness.executor.get_active_status()
    assert status is not None and status.is_terminal
    assert status.value == "unknown"
    assert harness.executor.active_invocation().stop_reason == "dex1_left_stale"


@pytest.mark.parametrize('stamp', [float('nan'), float('inf'), 1001.0])
def test_invalid_dex1_timestamp_never_reports_healthy(stamp):
    integration = Dex1Integration(clock=lambda: 1000.)
    integration.publish('left', opening=.5, received_at=stamp)
    assert not integration.readiness('left').ok
