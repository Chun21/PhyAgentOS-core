"""g1d_evidence_v1 evidence bundle and PAOS verification/confirmation.

This module connects G1_D Tool facts to PhyAgentOS acceptance: it collects
versioned before/during/after/confirmation artifacts linked to the task,
revision, binding, plan, and invocation identities, enforces the robot-state
acceptance criteria (bilateral 2 cm/5 deg held for 2 s, optional Dex1 opening
within 0.05 for 1 s), and requires an explicit confirmed operator record for
object-level acceptance in the no-camera stage.  A Tool result alone never
establishes task acceptance.
"""

from __future__ import annotations

import enum
import hashlib
import json
import math
import uuid
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
from typing import Any

from pydantic import ValidationError

from PhyAgentOS.skill_runtime.g1d_executor import ActionStatus
from PhyAgentOS.skill_runtime.g1d_planner import CartesianPose, quaternion_angle_deg
from PhyAgentOS.verification.contracts import CriterionVerdict, VerificationVerdict

EVIDENCE_BUNDLE_VERSION = "g1d_evidence_v1"

POSITION_TOLERANCE_M = 0.02
ORIENTATION_TOLERANCE_DEG = 5.0
POSE_HOLD_S = 2.0
DEX1_TOLERANCE = 0.05
DEX1_HOLD_S = 1.0
# Observations further apart than this break a hold window.
MAX_OBSERVATION_GAP_S = 0.2


class EvidenceError(RuntimeError):
    """Base class for explicit evidence-collection and acceptance errors."""


class EvidencePhase(enum.Enum):
    """Operation phases one evidence artifact can document."""

    BEFORE = "before"
    DURING = "during"
    AFTER = "after"
    CONFIRMATION = "confirmation"


class CriterionStatus(enum.Enum):
    """Status of one physical acceptance criterion."""

    SATISFIED = "satisfied"
    UNSATISFIED = "unsatisfied"
    UNKNOWN = "unknown"


class ConfirmationResult(enum.Enum):
    """Result of one explicit operator confirmation."""

    CONFIRMED = "confirmed"
    REJECTED = "rejected"
    UNCERTAIN = "uncertain"


class AcceptanceProfile(enum.Enum):
    """The two first-stage acceptance profiles."""

    ROBOT_STATE_REACHED = "robot_state_reached"
    OPERATOR_CONFIRMED_OBJECT_STATE = "operator_confirmed_object_state"


class G1DTaskOutcome(enum.Enum):
    """Task-level acceptance outcome; a Tool result alone is not acceptance."""

    SUCCESS = "success"
    FAILURE = "failure"
    CANCELLED = "cancelled"
    STOPPED = "stopped"
    DEADLINE_EXCEEDED = "deadline_exceeded"
    UNKNOWN = "unknown"
    INCONCLUSIVE = "inconclusive"





@dataclass(frozen=True)
class G1DEvidenceArtifact:
    """One content-addressed evidence record with provenance and times."""

    artifact_id: str
    phase: EvidencePhase
    kind: str
    provenance: str
    wall_clock_at: float
    monotonic_at: float
    sha256: str
    byte_size: int
    collection_error: str | None = None
    links: tuple[tuple[str, str], ...] = ()


@dataclass(frozen=True)
class OperatorConfirmation:
    """An explicit operator statement about an otherwise unobservable fact."""

    confirmer_id: str
    question: str
    result: ConfirmationResult
    wall_clock_at: float
    monotonic_at: float
    artifact_id: str = ""


@dataclass(frozen=True)
class G1DEvidenceBundle:
    """Versioned collection of evidence linked to one task execution."""

    version: str
    bundle_id: str
    task_id: str
    task_revision: int
    skill_version: str
    runtime_instance_id: str
    profile_digest: str
    plan_id: str
    invocation_id: str
    artifacts: tuple[G1DEvidenceArtifact, ...]
    created_wall_clock_at: float
    created_monotonic_at: float


class G1DEvidenceCollector:
    """Collect phase artifacts and build one g1d_evidence_v1 bundle."""

    def __init__(
        self,
        *,
        clock: Callable[[], float],
        wall_clock: Callable[[], float],
    ) -> None:
        self._clock = clock
        self._wall_clock = wall_clock
        self._artifacts: list[G1DEvidenceArtifact] = []

    def add(
        self,
        phase: EvidencePhase,
        *,
        kind: str,
        payload: Any,
        provenance: str = "g1d-runtime",
    ) -> G1DEvidenceArtifact:
        """Record one artifact; the payload is hashed and sized as given."""

        if isinstance(payload, bytes):
            raw = payload
        elif isinstance(payload, str):
            raw = payload.encode("utf-8")
        else:
            raw = json.dumps(payload, sort_keys=True, default=repr).encode("utf-8")
        digest = hashlib.sha256(raw).hexdigest()
        artifact = G1DEvidenceArtifact(
            # Content-addressed id: the digest prefix carries the content;
            # the sequence disambiguates identical payloads over time.
            artifact_id=f"ev-{len(self._artifacts):06d}-{digest[:12]}",
            phase=phase,
            kind=kind,
            provenance=provenance,
            wall_clock_at=self._wall_clock(),
            monotonic_at=self._clock(),
            sha256=hashlib.sha256(raw).hexdigest(),
            byte_size=len(raw),
        )
        self._artifacts.append(artifact)
        return artifact

    def capture(
        self,
        phase: EvidencePhase,
        *,
        kind: str,
        capture: Callable[[], Any],
        provenance: str = "g1d-runtime",
    ) -> G1DEvidenceArtifact:
        """Capture through a callable; failures become explicit error records."""

        try:
            payload = capture()
        except Exception as error:  # noqa: BLE001 - collection errors are evidence
            artifact = G1DEvidenceArtifact(
                artifact_id=f"ev-{len(self._artifacts):06d}-{hashlib.sha256(b'').hexdigest()[:12]}",
                phase=phase,
                kind=kind,
                provenance=provenance,
                wall_clock_at=self._wall_clock(),
                monotonic_at=self._clock(),
                sha256=hashlib.sha256(b"").hexdigest(),
                byte_size=0,
                collection_error=str(error),
            )
            self._artifacts.append(artifact)
            return artifact
        return self.add(phase, kind=kind, payload=payload, provenance=provenance)

    def record_confirmation(
        self,
        *,
        confirmer_id: str,
        question: str,
        result: str,
    ) -> OperatorConfirmation:
        """Record an explicit operator confirmation as a confirmation artifact."""

        if not isinstance(result, ConfirmationResult):
            raise EvidenceError("confirmation result must be a ConfirmationResult")
        confirmation = OperatorConfirmation(
            confirmer_id=confirmer_id,
            question=question,
            result=result,
            wall_clock_at=self._wall_clock(),
            monotonic_at=self._clock(),
        )
        artifact = self.add(
            EvidencePhase.CONFIRMATION,
            kind="operator_confirmation",
            payload={
                "confirmer_id": confirmer_id,
                "question": question,
                "result": result.value,
                "monotonic_at": confirmation.monotonic_at,
            },
            provenance="human-confirmation",
        )
        return replace(confirmation, artifact_id=artifact.artifact_id)

    def build(
        self,
        *,
        task_id: str,
        task_revision: int,
        skill_version: str,
        runtime_instance_id: str,
        profile_digest: str,
        plan_id: str,
        invocation_id: str,
    ) -> G1DEvidenceBundle:
        links = (
            ("task_id", task_id),
            ("task_revision", str(task_revision)),
            ("skill_version", skill_version),
            ("runtime_instance_id", runtime_instance_id),
            ("plan_id", plan_id),
            ("invocation_id", invocation_id),
        )
        linked = tuple(
            replace(artifact, links=links) for artifact in self._artifacts
        )
        return G1DEvidenceBundle(
            version=EVIDENCE_BUNDLE_VERSION,
            bundle_id=f"bundle-{uuid.uuid4().hex}",
            task_id=task_id,
            task_revision=task_revision,
            skill_version=skill_version,
            runtime_instance_id=runtime_instance_id,
            profile_digest=profile_digest,
            plan_id=plan_id,
            invocation_id=invocation_id,
            artifacts=linked,
            created_wall_clock_at=self._wall_clock(),
            created_monotonic_at=self._clock(),
        )


# ---------------------------------------------------------------------------
# Robot-state acceptance criteria
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PoseObservation:
    """One post-motion observation of both end effectors and the gate."""

    monotonic_at: float
    left_pose: CartesianPose
    right_pose: CartesianPose
    left_dex1_opening: float | None = None
    right_dex1_opening: float | None = None
    state_age_ms: float = 0.0
    safety_gate: str = "ready"
    state_crc_valid: bool = True
    artifact_id: str = ""


@dataclass(frozen=True)
class CriterionResult:
    """One acceptance criterion with status and evidence references."""

    name: str
    status: CriterionStatus
    evidence_refs: tuple[str, ...] = ()
    detail: str = ""

    @property
    def status_value(self) -> str:
        return self.status.value


def _hold_window_longest(
    observations: Sequence[PoseObservation],
    within: Callable[[PoseObservation], bool],
) -> tuple[float, list[str]]:
    """Longest contiguous in-tolerance run (seconds), gap-bounded.

    Observations further apart than MAX_OBSERVATION_GAP_S break the run:
    sparse sampling must not fabricate a held window.
    """

    ordered = sorted(observations, key=lambda item: item.monotonic_at)
    longest = 0.0
    best_refs: list[str] = []
    run: list[PoseObservation] = []

    def flush() -> None:
        nonlocal longest, best_refs
        if run and (run[-1].monotonic_at - run[0].monotonic_at) > longest:
            longest = run[-1].monotonic_at - run[0].monotonic_at
            best_refs = [item.artifact_id for item in run]

    previous_at: float | None = None
    for observation in ordered:
        if (
            previous_at is not None
            and observation.monotonic_at - previous_at > MAX_OBSERVATION_GAP_S
        ):
            flush()
            run = []
        previous_at = observation.monotonic_at
        if within(observation):
            run.append(observation)
        else:
            flush()
            run = []
    flush()
    return longest, best_refs


def evaluate_robot_state_criteria(
    target_left: CartesianPose,
    target_right: CartesianPose,
    observations: Sequence[PoseObservation],
    *,
    dex1_targets: Mapping[str, float | None] | None = None,
) -> list[CriterionResult]:
    """Enforce the robot-state acceptance profile over post-motion evidence."""

    dex1_targets = dict(dex1_targets or {})
    results: list[CriterionResult] = []

    if not observations:
        results.append(
            CriterionResult(
                name="bilateral_pose_within_tolerance",
                status=CriterionStatus.UNKNOWN,
                detail="no post-motion observations were collected",
            )
        )
        results.append(
            CriterionResult(
                name="state_fresh_and_safe",
                status=CriterionStatus.UNKNOWN,
                detail="no observations",
            )
        )
        if any(dex1_targets.get(side) is not None for side in ("left", "right")):
            results.append(
                CriterionResult(
                    name="dex1_opening_within_tolerance",
                    status=CriterionStatus.UNKNOWN,
                    detail="no observations",
                )
            )
        return results

    def pose_within(observation: PoseObservation) -> bool:
        left_pos_error = math.dist(observation.left_pose.position_m, target_left.position_m)
        right_pos_error = math.dist(observation.right_pose.position_m, target_right.position_m)
        left_ori_error = quaternion_angle_deg(
            observation.left_pose.orientation_xyzw, target_left.orientation_xyzw
        )
        right_ori_error = quaternion_angle_deg(
            observation.right_pose.orientation_xyzw, target_right.orientation_xyzw
        )
        return (
            left_pos_error <= POSITION_TOLERANCE_M
            and right_pos_error <= POSITION_TOLERANCE_M
            and left_ori_error <= ORIENTATION_TOLERANCE_DEG
            and right_ori_error <= ORIENTATION_TOLERANCE_DEG
        )

    longest, refs = _hold_window_longest(observations, pose_within)
    results.append(
        CriterionResult(
            name="bilateral_pose_within_tolerance",
            status=CriterionStatus.SATISFIED
            if longest >= POSE_HOLD_S
            else CriterionStatus.UNSATISFIED,
            evidence_refs=tuple(ref for ref in refs if ref),
            detail=(
                f"bilateral pose held within {POSITION_TOLERANCE_M * 100:.0f} cm/"
                f"{ORIENTATION_TOLERANCE_DEG:.0f} deg for {longest:.2f} s "
                f"(needs {POSE_HOLD_S:.0f} s)"
            ),
        )
    )

    gate_ok = all(
        item.state_age_ms <= 100.0
        and item.safety_gate == "ready"
        and item.state_crc_valid
        for item in observations
    )
    results.append(
        CriterionResult(
            name="state_fresh_and_safe",
            status=CriterionStatus.SATISFIED if gate_ok else CriterionStatus.UNSATISFIED,
            evidence_refs=tuple(
                item.artifact_id for item in observations if item.artifact_id
            ),
            detail="all observations are fresh and the safety gate is ready"
            if gate_ok
            else "at least one observation is stale or the safety gate is not ready",
        )
    )

    if any(dex1_targets.get(side) is not None for side in ("left", "right")):
        def dex1_within(observation: PoseObservation) -> bool:
            for side in ("left", "right"):
                target = dex1_targets.get(side)
                actual = (
                    observation.left_dex1_opening
                    if side == "left"
                    else observation.right_dex1_opening
                )
                if target is None:
                    continue
                if actual is None or abs(actual - target) > DEX1_TOLERANCE:
                    return False
            return True

        longest, refs = _hold_window_longest(observations, dex1_within)
        results.append(
            CriterionResult(
                name="dex1_opening_within_tolerance",
                status=CriterionStatus.SATISFIED
                if longest >= DEX1_HOLD_S
                else CriterionStatus.UNSATISFIED,
                evidence_refs=tuple(ref for ref in refs if ref),
                detail=(
                    f"requested Dex1 opening held within {DEX1_TOLERANCE} for "
                    f"{longest:.2f} s (needs {DEX1_HOLD_S:.0f} s)"
                ),
            )
        )

    return results


# ---------------------------------------------------------------------------
# Task acceptance: lifecycle mapping and operator confirmation
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class G1DAcceptanceResult:
    """Task-level acceptance outcome with criteria and confirmation record."""

    outcome: G1DTaskOutcome
    criteria: tuple[CriterionResult, ...]
    confirmation: OperatorConfirmation | None
    profile: str
    reason: str

    def to_paos_verdict(self) -> dict[str, Any]:
        """Map to the PAOS verification vocabulary (success/failure/inconclusive).

        deadline_exceeded, cancelled, stopped, and unknown stay inconclusive at
        the PAOS layer: they are distinct G1_D lifecycle outcomes, not evidence
        of a violated criterion.
        """

        verdict = {
            G1DTaskOutcome.SUCCESS: "success",
            G1DTaskOutcome.FAILURE: "failure",
        }.get(self.outcome, "inconclusive")
        return {
            "verdict": verdict,
            "criteria": [
                CriterionVerdict(
                    criterion=criterion.name,
                    status=criterion.status.value,  # type: ignore[arg-type]
                    evidence_refs=list(criterion.evidence_refs),
                )
                for criterion in self.criteria
            ],
            "reason": self.reason,
        }

    def to_verification_verdict(self) -> VerificationVerdict | None:
        """Construct a PAOS VerificationVerdict, or None when it would not validate.

        A lifecycle failure with all-satisfied criteria (for example a Safety
        fault after reaching the target) cannot be represented as a PAOS
        failure verdict; the caller keeps the G1_D outcome instead.
        """

        mapped = self.to_paos_verdict()
        try:
            return VerificationVerdict(
                verdict=mapped["verdict"],  # type: ignore[arg-type]
                criteria=mapped["criteria"],
                reason=self.reason,
                lesson=self.reason,
            )
        except ValidationError:
            return None


_TERMINAL_OUTCOME_BY_STATUS = {
    ActionStatus.CANCELLED: G1DTaskOutcome.CANCELLED,
    ActionStatus.STOPPED: G1DTaskOutcome.STOPPED,
    ActionStatus.DEADLINE_EXCEEDED: G1DTaskOutcome.DEADLINE_EXCEEDED,
    ActionStatus.UNKNOWN: G1DTaskOutcome.UNKNOWN,
    ActionStatus.FAILED: G1DTaskOutcome.FAILURE,
}


def evaluate_acceptance(
    *,
    invocation_status: ActionStatus,
    criteria: Sequence[CriterionResult],
    confirmation: OperatorConfirmation | None = None,
    profile: AcceptanceProfile = AcceptanceProfile.ROBOT_STATE_REACHED,
) -> G1DAcceptanceResult:
    """Map one terminated Action plus evidence into a task-level outcome."""

    if not isinstance(profile, AcceptanceProfile):
        raise EvidenceError("profile must be an AcceptanceProfile")

    if invocation_status in _TERMINAL_OUTCOME_BY_STATUS:
        outcome = _TERMINAL_OUTCOME_BY_STATUS[invocation_status]
        reason = f"invocation terminated as {invocation_status.value}"
        return G1DAcceptanceResult(
            outcome=outcome,
            criteria=tuple(criteria),
            confirmation=confirmation,
            profile=profile,
            reason=reason,
        )

    # SUCCEEDED: the criteria decide; anything unresolved stays inconclusive.
    statuses = {criterion.status for criterion in criteria}
    if not statuses or CriterionStatus.UNKNOWN in statuses:
        outcome = G1DTaskOutcome.INCONCLUSIVE
        reason = "at least one criterion is unknown: evidence is insufficient"
    elif statuses == {CriterionStatus.SATISFIED}:
        outcome = G1DTaskOutcome.SUCCESS
        reason = "every declared criterion is satisfied with evidence"
    else:
        outcome = G1DTaskOutcome.FAILURE
        reason = "at least one declared criterion is unsatisfied"

    if profile is AcceptanceProfile.OPERATOR_CONFIRMED_OBJECT_STATE:
        if confirmation is None:
            return G1DAcceptanceResult(
                outcome=G1DTaskOutcome.INCONCLUSIVE,
                criteria=tuple(criteria),
                confirmation=None,
                profile=profile,
                reason="object-level acceptance requires an explicit operator record",
            )
        if confirmation.result is ConfirmationResult.REJECTED:
            return G1DAcceptanceResult(
                outcome=G1DTaskOutcome.FAILURE,
                criteria=tuple(criteria),
                confirmation=confirmation,
                profile=profile,
                reason=f"operator {confirmation.confirmer_id} rejected the object condition",
            )
        if confirmation.result is ConfirmationResult.UNCERTAIN:
            return G1DAcceptanceResult(
                outcome=G1DTaskOutcome.INCONCLUSIVE,
                criteria=tuple(criteria),
                confirmation=confirmation,
                profile=profile,
                reason="operator could not establish the object condition",
            )
        if outcome is not G1DTaskOutcome.SUCCESS:
            return G1DAcceptanceResult(
                outcome=outcome,
                criteria=tuple(criteria),
                confirmation=confirmation,
                profile=profile,
                reason=f"operator confirmed, but robot-state criteria are not all satisfied ({reason})",
            )

    return G1DAcceptanceResult(
        outcome=outcome,
        criteria=tuple(criteria),
        confirmation=confirmation,
        profile=profile,
        reason=reason,
    )


__all__ = [
    "AcceptanceProfile",
    "ConfirmationResult",
    "CriterionResult",
    "CriterionStatus",
    "DEX1_HOLD_S",
    "DEX1_TOLERANCE",
    "EvidenceError",
    "EvidencePhase",
    "G1DAcceptanceResult",
    "G1DEvidenceArtifact",
    "G1DEvidenceBundle",
    "G1DEvidenceCollector",
    "G1DTaskOutcome",
    "OperatorConfirmation",
    "POSE_HOLD_S",
    "PoseObservation",
    "evaluate_acceptance",
    "evaluate_robot_state_criteria",
]
