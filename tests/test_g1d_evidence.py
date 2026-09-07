from __future__ import annotations

import hashlib
import math
from typing import Any

import pytest

from PhyAgentOS.skill_runtime.g1d_evidence import (
    AcceptanceProfile,
    ConfirmationResult,
    CriterionResult,
    CriterionStatus,
    DEX1_HOLD_S,
    DEX1_TOLERANCE,
    EvidencePhase,
    G1DEvidenceCollector,
    G1DTaskOutcome,
    OperatorConfirmation,
    PoseObservation,
    evaluate_acceptance,
    evaluate_robot_state_criteria,
    quaternion_angle_deg,
)
from PhyAgentOS.skill_runtime.g1d_executor import ActionStatus
from PhyAgentOS.skill_runtime.g1d_planner import CartesianPose


class Clock:
    def __init__(self, value: float = 2000.0) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def _pose(position=(0.05, 0.25, 0.10)) -> CartesianPose:
    return CartesianPose(position_m=position, orientation_xyzw=(0.0, 0.0, 0.0, 1.0))


def _observation(
    t: float,
    *,
    left: CartesianPose | None = None,
    right: CartesianPose | None = None,
    left_dex1: float | None = None,
    right_dex1: float | None = None,
    state_age_ms: float = 1.0,
    safety_gate: str = "ready",
    artifact_id: str = "obs",
) -> PoseObservation:
    return PoseObservation(
        monotonic_at=t,
        left_pose=left if left is not None else _pose(),
        right_pose=right if right is not None else _pose((0.05, -0.25, 0.10)),
        left_dex1_opening=left_dex1,
        right_dex1_opening=right_dex1,
        state_age_ms=state_age_ms,
        safety_gate=safety_gate,
        artifact_id=artifact_id,
    )


# ---------------------------------------------------------------------------
# g1d_evidence_v1 bundle
# ---------------------------------------------------------------------------


def test_bundle_links_task_revision_binding_plan_and_invocation_across_phases() -> None:
    clock = Clock()
    collector = G1DEvidenceCollector(clock=clock, wall_clock=lambda: 1_700_000_000.0)

    before = collector.add(
        EvidencePhase.BEFORE, kind="state_snapshot", payload={"state_age_ms": 0.5}
    )
    during = collector.add(EvidencePhase.DURING, kind="stream_fact", payload={"seq": 7})
    after = collector.add(EvidencePhase.AFTER, kind="state_snapshot", payload={"frame": 43})
    collector.record_confirmation(
        confirmer_id="operator-a",
        question="Is the object in the tray?",
        result=ConfirmationResult.CONFIRMED,
    )

    bundle = collector.build(
        task_id="task-1",
        task_revision=3,
        skill_version="0.1.0",
        runtime_instance_id="runtime-test",
        profile_digest="digest-abc",
        plan_id="plan-1",
        invocation_id="inv-1",
    )

    assert bundle.version == "g1d_evidence_v1"
    assert bundle.bundle_id
    assert bundle.task_id == "task-1"
    assert bundle.task_revision == 3
    assert bundle.skill_version == "0.1.0"
    assert bundle.runtime_instance_id == "runtime-test"
    assert bundle.profile_digest == "digest-abc"
    assert bundle.plan_id == "plan-1"
    assert bundle.invocation_id == "inv-1"
    phases = [artifact.phase for artifact in bundle.artifacts]
    assert [p.value for p in phases] == ["before", "during", "after", "confirmation"]
    assert before.artifact_id in [a.artifact_id for a in bundle.artifacts]
    assert during.provenance and after.provenance


def test_artifacts_carry_hash_size_times_provenance_and_collection_errors() -> None:
    clock = Clock()
    wall = Clock(1_700_000_000.0)
    collector = G1DEvidenceCollector(clock=clock, wall_clock=wall)
    payload = '{"state_age_ms": 0.5}'

    artifact = collector.add(EvidencePhase.BEFORE, kind="state_snapshot", payload=payload)

    assert artifact.sha256 == hashlib.sha256(payload.encode()).hexdigest()
    assert artifact.byte_size == len(payload.encode())
    assert artifact.wall_clock_at == 1_700_000_000.0
    assert artifact.monotonic_at == 2000.0
    wall.advance(5.0)
    clock.advance(0.25)
    later = collector.add(EvidencePhase.DURING, kind="stream_fact", payload="{}")
    assert later.wall_clock_at > artifact.wall_clock_at
    assert later.monotonic_at > artifact.monotonic_at

    def boom() -> Any:
        raise RuntimeError("sensor bus offline")

    failed = collector.capture(EvidencePhase.AFTER, kind="state_snapshot", capture=boom)
    assert failed.collection_error == "sensor bus offline"
    assert failed.byte_size == 0


# ---------------------------------------------------------------------------
# Robot-state criteria: 2 cm / 5 deg for 2 s, optional Dex1 0.05 for 1 s
# ---------------------------------------------------------------------------


def test_pose_criteria_distinguish_satisfied_unsatisfied_and_unknown() -> None:
    target_left = _pose()
    target_right = _pose((0.05, -0.25, 0.10))

    # Held within 2 cm/5 deg for over two seconds: satisfied.
    held = [_observation(10.0 + 0.1 * i, artifact_id=f"ok-{i}") for i in range(22)]
    criteria = {
        c.name: c
        for c in evaluate_robot_state_criteria(target_left, target_right, held)
    }
    assert criteria["bilateral_pose_within_tolerance"].status is CriterionStatus.SATISFIED
    assert criteria["bilateral_pose_within_tolerance"].evidence_refs
    assert criteria["state_fresh_and_safe"].status is CriterionStatus.SATISFIED

    # Never within tolerance after motion: unsatisfied.
    far = [
        _observation(10.0 + 0.1 * i, left=_pose((0.5, 0.25, 0.10)), artifact_id=f"far-{i}")
        for i in range(22)
    ]
    criteria = {
        c.name: c
        for c in evaluate_robot_state_criteria(target_left, target_right, far)
    }
    assert criteria["bilateral_pose_within_tolerance"].status is CriterionStatus.UNSATISFIED

    # No post-motion observations at all: unknown.
    criteria = {
        c.name: c for c in evaluate_robot_state_criteria(target_left, target_right, [])
    }
    assert criteria["bilateral_pose_within_tolerance"].status is CriterionStatus.UNKNOWN
    assert criteria["state_fresh_and_safe"].status is CriterionStatus.UNKNOWN


def test_orientation_error_uses_quaternion_angle_in_degrees() -> None:
    identity = (0.0, 0.0, 0.0, 1.0)
    rotated = (math.sin(math.radians(3.0)), 0.0, 0.0, math.cos(math.radians(3.0)))
    assert quaternion_angle_deg(identity, identity) == pytest.approx(0.0, abs=1e-9)
    assert quaternion_angle_deg(identity, rotated) == pytest.approx(6.0, abs=1e-6)


def test_dex1_criterion_is_optional_and_enforces_0_05_for_1s() -> None:
    target_left = _pose()
    target_right = _pose((0.05, -0.25, 0.10))

    # Not requested: no Dex1 criterion at all.
    plain = evaluate_robot_state_criteria(target_left, target_right, [_observation(10.0)])
    assert not any("dex1" in c.name for c in plain)

    # Requested and held within 0.05 for over a second: satisfied.
    good = [
        _observation(10.0 + 0.1 * i, left_dex1=0.60, right_dex1=0.20, artifact_id=f"d-{i}")
        for i in range(12)
    ]
    criteria = {c.name: c for c in evaluate_robot_state_criteria(
        target_left, target_right, good, dex1_targets={"left": 0.62, "right": 0.18}
    )}
    assert criteria["dex1_opening_within_tolerance"].status is CriterionStatus.SATISFIED
    assert DEX1_TOLERANCE == 0.05 and DEX1_HOLD_S == 1.0

    # Requested but drifting beyond 0.05: unsatisfied.
    drift = [
        _observation(10.0 + 0.1 * i, left_dex1=0.9, right_dex1=0.2, artifact_id=f"x-{i}")
        for i in range(12)
    ]
    criteria = {c.name: c for c in evaluate_robot_state_criteria(
        target_left, target_right, drift, dex1_targets={"left": 0.62, "right": 0.18}
    )}
    assert criteria["dex1_opening_within_tolerance"].status is CriterionStatus.UNSATISFIED


# ---------------------------------------------------------------------------
# Acceptance outcomes and lifecycle mapping
# ---------------------------------------------------------------------------


def _criteria(statuses: dict[str, CriterionStatus]):
    return [
        CriterionResult(name=name, status=status, evidence_refs=(f"ref-{name}",), detail="")
        for name, status in statuses.items()
    ]


def test_lifecycle_status_maps_to_task_outcomes() -> None:
    ok = _criteria({"bilateral_pose_within_tolerance": CriterionStatus.SATISFIED, "state_fresh_and_safe": CriterionStatus.SATISFIED})
    assert (
        evaluate_acceptance(
            invocation_status=ActionStatus.SUCCEEDED, criteria=ok
        ).outcome
        is G1DTaskOutcome.SUCCESS
    )

    unmet = _criteria({"bilateral_pose_within_tolerance": CriterionStatus.UNSATISFIED})
    assert (
        evaluate_acceptance(invocation_status=ActionStatus.SUCCEEDED, criteria=unmet).outcome
        is G1DTaskOutcome.FAILURE
    )
    unknown = _criteria({"bilateral_pose_within_tolerance": CriterionStatus.UNKNOWN})
    assert (
        evaluate_acceptance(invocation_status=ActionStatus.SUCCEEDED, criteria=unknown).outcome
        is G1DTaskOutcome.INCONCLUSIVE
    )
    assert (
        evaluate_acceptance(invocation_status=ActionStatus.CANCELLED, criteria=ok).outcome
        is G1DTaskOutcome.CANCELLED
    )
    assert (
        evaluate_acceptance(invocation_status=ActionStatus.STOPPED, criteria=ok).outcome
        is G1DTaskOutcome.STOPPED
    )
    assert (
        evaluate_acceptance(
            invocation_status=ActionStatus.DEADLINE_EXCEEDED, criteria=ok
        ).outcome
        is G1DTaskOutcome.DEADLINE_EXCEEDED
    )
    assert (
        evaluate_acceptance(invocation_status=ActionStatus.UNKNOWN, criteria=ok).outcome
        is G1DTaskOutcome.UNKNOWN
    )
    assert (
        evaluate_acceptance(invocation_status=ActionStatus.FAILED, criteria=ok).outcome
        is G1DTaskOutcome.FAILURE
    )


def test_object_state_requires_explicit_confirmed_operator_record() -> None:
    ok = _criteria({"bilateral_pose_within_tolerance": CriterionStatus.SATISFIED})

    confirmed = OperatorConfirmation(
        confirmer_id="operator-a",
        question="Is the object in the tray?",
        result=ConfirmationResult.CONFIRMED,
        wall_clock_at=1_700_000_010.0,
        monotonic_at=2010.0,
    )
    result = evaluate_acceptance(
        invocation_status=ActionStatus.SUCCEEDED,
        criteria=ok,
        confirmation=confirmed,
        profile=AcceptanceProfile.OPERATOR_CONFIRMED_OBJECT_STATE,
    )
    assert result.outcome is G1DTaskOutcome.SUCCESS

    rejected = OperatorConfirmation(
        confirmer_id="operator-a",
        question="Is the object in the tray?",
        result=ConfirmationResult.REJECTED,
        wall_clock_at=1_700_000_010.0,
        monotonic_at=2010.0,
    )
    result = evaluate_acceptance(
        invocation_status=ActionStatus.SUCCEEDED,
        criteria=ok,
        confirmation=rejected,
        profile=AcceptanceProfile.OPERATOR_CONFIRMED_OBJECT_STATE,
    )
    assert result.outcome is G1DTaskOutcome.FAILURE

    uncertain = OperatorConfirmation(
        confirmer_id="operator-a",
        question="Is the object in the tray?",
        result=ConfirmationResult.UNCERTAIN,
        wall_clock_at=1_700_000_010.0,
        monotonic_at=2010.0,
    )
    assert (
        evaluate_acceptance(
            invocation_status=ActionStatus.SUCCEEDED,
            criteria=ok,
            confirmation=uncertain,
            profile=AcceptanceProfile.OPERATOR_CONFIRMED_OBJECT_STATE,
        ).outcome
        is G1DTaskOutcome.INCONCLUSIVE
    )
    # Missing record entirely: inconclusive, never success.
    assert (
        evaluate_acceptance(
            invocation_status=ActionStatus.SUCCEEDED,
            criteria=ok,
            profile=AcceptanceProfile.OPERATOR_CONFIRMED_OBJECT_STATE,
        ).outcome
        is G1DTaskOutcome.INCONCLUSIVE
    )
    # A rejection is a failure even when robot state was fine.
    ok_state = _criteria({"state_fresh_and_safe": CriterionStatus.SATISFIED})
    assert (
        evaluate_acceptance(
            invocation_status=ActionStatus.SUCCEEDED,
            criteria=ok_state,
            confirmation=rejected,
            profile=AcceptanceProfile.OPERATOR_CONFIRMED_OBJECT_STATE,
        ).outcome
        is G1DTaskOutcome.FAILURE
    )


def test_paos_verification_connection() -> None:
    from PhyAgentOS.verification.contracts import CriterionVerdict

    ok = _criteria(
        {
            "bilateral_pose_within_tolerance": CriterionStatus.SATISFIED,
            "state_fresh_and_safe": CriterionStatus.SATISFIED,
        }
    )
    result = evaluate_acceptance(invocation_status=ActionStatus.SUCCEEDED, criteria=ok)

    paos = result.to_paos_verdict()

    assert paos["verdict"] == "success"
    assert all(isinstance(c, CriterionVerdict) for c in paos["criteria"])
    assert paos["criteria"][0].status == "satisfied"  # PAOS Literal vocabulary
    assert paos["criteria"][0].evidence_refs == ["ref-bilateral_pose_within_tolerance"]

    unmet = _criteria({"bilateral_pose_within_tolerance": CriterionStatus.UNSATISFIED})
    result = evaluate_acceptance(invocation_status=ActionStatus.SUCCEEDED, criteria=unmet)
    assert result.to_paos_verdict()["verdict"] == "failure"

    stopped = evaluate_acceptance(invocation_status=ActionStatus.STOPPED, criteria=ok)
    assert stopped.to_paos_verdict()["verdict"] == "inconclusive"


def test_sparse_observations_do_not_fabricate_a_hold_window() -> None:
    target_left = _pose()
    target_right = _pose((0.05, -0.25, 0.10))
    # Two in-tolerance observations 2 s apart with nothing between: the gap
    # must break the window instead of counting as a held 2 s.
    sparse = [_observation(10.0), _observation(12.0)]
    criteria = {
        c.name: c
        for c in evaluate_robot_state_criteria(target_left, target_right, sparse)
    }
    assert criteria["bilateral_pose_within_tolerance"].status is CriterionStatus.UNSATISFIED


def test_verification_verdict_constructs_the_paos_model() -> None:
    from PhyAgentOS.verification.contracts import VerificationVerdict

    ok = _criteria({"state_fresh_and_safe": CriterionStatus.SATISFIED})
    result = evaluate_acceptance(invocation_status=ActionStatus.SUCCEEDED, criteria=ok)
    verdict = result.to_verification_verdict()
    assert isinstance(verdict, VerificationVerdict)
    assert verdict.verdict == "success"

    # A lifecycle failure with all-satisfied criteria cannot be a PAOS
    # failure verdict: the constructor returns None and the G1_D outcome
    # remains authoritative.
    failed = evaluate_acceptance(invocation_status=ActionStatus.FAILED, criteria=ok)
    assert failed.outcome is G1DTaskOutcome.FAILURE
    assert failed.to_verification_verdict() is None
