"""Robot-free conformance harness for the g1d-manipulation Skill.

Every scenario in this module runs the full offline stack — bundle discovery,
Tool schemas, planner, executor, Dex1 integration, evidence, installer —
against fake DDS sources, fixture kinematics, and a controllable clock.  No
scenario imports an SDK, opens a DDS domain, or connects to a robot; the same
scenarios document the acceptance ladder's offline level.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest

from PhyAgentOS.skill_runtime.archive import ArchiveValidator, sha256_file
from PhyAgentOS.skill_runtime.g1d_adapter import G1DAdapter, make_lowstate_frame
from PhyAgentOS.skill_runtime.g1d_dex1 import Dex1Integration
from PhyAgentOS.skill_runtime.g1d_evidence import (
    AcceptanceProfile,
    CriterionStatus,
    EvidencePhase,
    G1DEvidenceCollector,
    PoseObservation,
    evaluate_acceptance,
    evaluate_robot_state_criteria,
)
from PhyAgentOS.skill_runtime.g1d_executor import (
    ActionStatus,
    ActiveActionError,
    BindingMismatchError,
    G1DExecutor,
    RecordingSink,
)
from PhyAgentOS.skill_runtime.g1d_planner import (
    CartesianPose,
    FixtureKinematics,
    G1DPlanner,
    PlanExpiredError,
    PoseValidationError,
    digest_json,
    load_kinematics_profile,
)
from PhyAgentOS.skill_runtime.installer import SkillInstaller
from PhyAgentOS.skill_runtime.manifest import load_manifest
from PhyAgentOS.skill_runtime.mock_runtime import MockSkillRuntime
from PhyAgentOS.skill_runtime.state import RuntimeStateStore
from scripts.package_skill import package

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
    def __init__(self, value: float = 3000.0) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def _home_targets() -> tuple[CartesianPose, CartesianPose]:
    """The bilateral home targets as FK-comparable poses."""

    return (
        CartesianPose(
            position_m=tuple(HOME_LEFT["position_m"]),  # type: ignore[arg-type]
            orientation_xyzw=tuple(HOME_LEFT["orientation_xyzw"]),  # type: ignore[arg-type]
        ),
        CartesianPose(
            position_m=tuple(HOME_RIGHT["position_m"]),  # type: ignore[arg-type]
            orientation_xyzw=tuple(HOME_RIGHT["orientation_xyzw"]),  # type: ignore[arg-type]
        ),
    )


class ConformanceHarness:
    """One robot-free runtime instance: fake DDS + fixtures + offline clock."""

    def __init__(
        self,
        *,
        runtime_instance_id: str = "runtime-conf",
        planner: G1DPlanner | None = None,
    ) -> None:
        self.clock = Clock()
        self.adapter = G1DAdapter(
            clock=self.clock, max_state_age_s=0.1, approved_mode_machine=7
        )
        profile = load_kinematics_profile(KINEMATICS)
        self.profile_digest = digest_json(profile)
        self.kinematics = FixtureKinematics(
            fixtures=[(dict(HOME_LEFT), dict(HOME_RIGHT), HOME_LEFT_Q, HOME_RIGHT_Q)]
        )
        self.planner = planner or G1DPlanner(
            kinematics=self.kinematics,
            clock=self.clock,
            skill_version="0.1.0",
            runtime_instance_id=runtime_instance_id,
            profile_digest=self.profile_digest,
        )
        self.dex1 = Dex1Integration(clock=self.clock)
        self.sink = RecordingSink()
        self.collector = G1DEvidenceCollector(
            clock=self.clock, wall_clock=lambda: 1_700_100_000.0
        )
        self.executor = G1DExecutor(
            adapter=self.adapter,
            planner=self.planner,
            clock=self.clock,
            sink=self.sink,
            skill_version="0.1.0",
            runtime_instance_id=runtime_instance_id,
            profile_digest=self.profile_digest,
            dex1=self.dex1,
        )

    # -- fake DDS publishers ------------------------------------------------

    def publish_arm(self) -> None:
        self.adapter.ingest(
            make_lowstate_frame(mode_machine=7, tick=42, positions=ARM_START, mode=[1] * 35),
            received_at=self.clock(),
        )

    def publish_dex1(self, side: str, opening: float) -> None:
        self.dex1.publish(side, opening=opening, received_at=self.clock())

    def make_plan(self):
        return self.planner.plan_pose(left=HOME_LEFT, right=HOME_RIGHT)

    # -- lifecycle driving ---------------------------------------------------

    def run_to_terminal(self, *, max_cycles: int = 200_000) -> ActionStatus:
        for _ in range(max_cycles):
            status = self.executor.get_active_status()
            if status is not None and status.is_terminal:
                return status
            self.publish_arm()
            self.clock.advance(0.002)
            self.executor.tick()
        raise AssertionError("invocation never reached a terminal status")

    def start_and_stream(self, cycles: int) -> None:
        self.publish_arm()
        for _ in range(cycles):
            self.publish_arm()
            self.clock.advance(0.002)
            self.executor.tick()


# ---------------------------------------------------------------------------
# Discovery / context / Query
# ---------------------------------------------------------------------------


def test_conformance_discovery_and_context() -> None:
    runtime = MockSkillRuntime(BUNDLE)
    runtime.start("real-g1d")

    discovery = runtime.gateway_tools()
    assert discovery["ok"] is True
    specs = json.loads(TOOLS.read_text())
    discovered_ids = {item["tool_id"] for item in discovery["data"]["tools"]}
    assert discovered_ids == set(specs)
    for item in discovery["data"]["tools"]:
        assert item["input_schema"] == specs[item["tool_id"]]["input_schema"]

    context = runtime.gateway_context("g1d.dual_arm.execute_pose")
    assert context["ok"] is True
    assert context["data"]["ready"] is True
    # Readiness-versus-runtime state: runtime health is not action readiness.
    assert runtime.status().runtime_ready is True
    assert runtime.status().action_ready is False


def test_conformance_queries_are_read_only_and_validated() -> None:
    harness = ConformanceHarness()
    harness.publish_arm()

    plan = harness.make_plan()
    assert plan.checks_passed
    state = harness.executor.query_state()
    assert state["safety_gate"] == "ready"
    assert state["dex1"]["left"]["status"] == "absent"

    # Invalid targets are rejected without any motion side effects.
    with pytest.raises(PoseValidationError):
        harness.planner.plan_pose(
            left={**HOME_LEFT, "position_m": [500.0, 0.0, 0.0]}, right=HOME_RIGHT
        )
    assert harness.sink.samples == []


def test_conformance_protected_configuration_is_injected_not_baked() -> None:
    manifest = load_manifest(BUNDLE / "skill.yaml")
    profile = manifest.profiles["real-g1d"]

    # Host/DDS/URDF/IK configuration is required from the protected
    # environment; the RuntimeManager preflight refuses to start without it.
    assert set(profile.required_environment) >= {
        "PAOS_G1D_DDS_DOMAIN",
        "PAOS_G1D_URDF_PATH",
        "PAOS_G1D_IK_CONFIG",
    }
    # Nothing baked into the profile environment carries host paths or
    # credentials; the Gateway binds loopback by the dataflow, not by config.
    for value in profile.environment.values():
        assert "192.168" not in value
        assert "/" not in value or value in {"rt/lowcmd", "rt/lowstate"}


# ---------------------------------------------------------------------------
# Action lifecycle, cancel/stop, deadline/unknown, concurrency, binding
# ---------------------------------------------------------------------------


def test_conformance_action_lifecycle_succeeds() -> None:
    harness = ConformanceHarness()
    harness.publish_arm()
    harness.collector.add(
        EvidencePhase.BEFORE, kind="state_snapshot", payload=harness.executor.query_state()
    )
    plan = harness.make_plan()
    harness.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")

    status = harness.run_to_terminal()

    assert status is ActionStatus.SUCCEEDED
    harness.collector.add(
        EvidencePhase.DURING, kind="stream_fact", payload={"frames": len(harness.sink.samples)}
    )
    for probe in (harness.sink.samples[0], harness.sink.samples[-1]):
        assert probe.frame.crc == probe.frame.calculated_crc()


def test_conformance_cancel_and_stop() -> None:
    harness = ConformanceHarness()
    harness.publish_arm()
    plan = harness.make_plan()

    # Pre-effect stop cancels.
    invocation = harness.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    assert harness.executor.stop(invocation_id=invocation.invocation_id) == "accepted"
    assert harness.run_to_terminal() is ActionStatus.CANCELLED

    # Operator stop during motion ends stopped and is idempotent.
    harness2 = ConformanceHarness()
    harness2.publish_arm()
    plan2 = harness2.make_plan()
    invocation2 = harness2.executor.execute_pose(plan_id=plan2.plan_id, caller_id="agent-1")
    harness2.start_and_stream(20)
    assert harness2.executor.stop(invocation_id=invocation2.invocation_id) == "accepted"
    assert harness2.executor.stop(invocation_id=invocation2.invocation_id) == "accepted"
    assert harness2.run_to_terminal() is ActionStatus.STOPPED


def test_conformance_deadline_and_unknown() -> None:
    harness = ConformanceHarness()
    harness.publish_arm()
    plan = harness.make_plan()
    harness.executor.execute_pose(
        plan_id=plan.plan_id, caller_id="agent-1", operation_deadline_s=1.0
    )
    assert harness.run_to_terminal() is ActionStatus.DEADLINE_EXCEEDED

    # Transport loss marks the invocation unknown and never replays blindly.
    harness2 = ConformanceHarness()
    harness2.publish_arm()
    plan2 = harness2.make_plan()
    invocation2 = harness2.executor.execute_pose(plan_id=plan2.plan_id, caller_id="agent-1")
    harness2.start_and_stream(20)
    harness2.executor.mark_unknown(invocation2.invocation_id, reason="transport_loss")
    assert harness2.executor.get_active_status() is ActionStatus.UNKNOWN


def test_conformance_concurrency_and_idempotent_retry() -> None:
    harness = ConformanceHarness()
    harness.publish_arm()
    plan = harness.make_plan()

    first = harness.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    retry = harness.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    assert retry.invocation_id == first.invocation_id

    with pytest.raises(ActiveActionError):
        harness.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-2")


def test_conformance_binding_rejects_replayed_plans() -> None:
    harness = ConformanceHarness()
    harness.publish_arm()
    plan = harness.make_plan()

    # A runtime restart changes the runtime identity: the old plan must be
    # rejected even though the (persisted) planner still holds it.
    restarted = ConformanceHarness(
        runtime_instance_id="runtime-restarted", planner=harness.planner
    )
    restarted.publish_arm()

    with pytest.raises(BindingMismatchError):
        restarted.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")


def test_conformance_crash_reconciliation_never_replays() -> None:
    crashed = ConformanceHarness()
    crashed.publish_arm()
    plan = crashed.make_plan()
    invocation = crashed.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    crashed.start_and_stream(20)

    # The crashed runtime's invocation is reconciled as unknown, not stopped.
    crashed.executor.mark_unknown(invocation.invocation_id, reason="runtime_crash")

    # A fresh runtime knows nothing about the old plan: replay is rejected.
    fresh = ConformanceHarness(runtime_instance_id="runtime-fresh")
    fresh.publish_arm()
    with pytest.raises(PlanExpiredError):
        fresh.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    assert fresh.sink.samples == []


# ---------------------------------------------------------------------------
# Evidence
# ---------------------------------------------------------------------------


def test_conformance_evidence_bundle_and_acceptance() -> None:
    harness = ConformanceHarness()
    harness.publish_arm()
    before = harness.collector.add(
        EvidencePhase.BEFORE, kind="state_snapshot", payload=harness.executor.query_state()
    )
    plan = harness.make_plan()
    harness.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    harness.collector.add(
        EvidencePhase.DURING, kind="stream_fact", payload={"caller": "agent-1"}
    )
    assert harness.run_to_terminal() is ActionStatus.SUCCEEDED

    # Post-motion observations: the FK surrogate maps the final commanded
    # joints back onto the requested target poses.
    final = harness.sink.samples[-1].frame
    left_q = [final.motor_cmd[slot].q for slot in range(15, 22)]
    right_q = [final.motor_cmd[slot].q for slot in range(22, 29)]
    fk_left, fk_right = harness.kinematics.solve_fk(left_q, right_q)
    observations = []
    for index in range(22):
        artifact = harness.collector.add(
            EvidencePhase.AFTER, kind="pose_observation", payload={"sample": index}
        )
        harness.clock.advance(0.1)
        observations.append(
            PoseObservation(
                monotonic_at=harness.clock(),
                left_pose=fk_left,
                right_pose=fk_right,
                artifact_id=artifact.artifact_id,
            )
        )
    harness.collector.add(
        EvidencePhase.AFTER, kind="state_snapshot", payload=harness.executor.query_state()
    )

    criteria = evaluate_robot_state_criteria(*_home_targets(), observations)
    assert all(criterion.status is CriterionStatus.SATISFIED for criterion in criteria)

    result = evaluate_acceptance(
        invocation_status=ActionStatus.SUCCEEDED, criteria=criteria
    )
    assert result.outcome.value == "success"
    assert result.to_verification_verdict() is not None

    bundle = harness.collector.build(
        task_id="task-conf",
        task_revision=1,
        skill_version="0.1.0",
        runtime_instance_id="runtime-conf",
        profile_digest=harness.profile_digest,
        plan_id=plan.plan_id,
        invocation_id=harness.executor.active_invocation().invocation_id,
    )
    assert bundle.version == "g1d_evidence_v1"
    phases = {artifact.phase.value for artifact in bundle.artifacts}
    assert phases >= {"before", "during", "after"}
    # Every artifact inside the bundle carries the execution identity links.
    linked = next(a for a in bundle.artifacts if a.artifact_id == before.artifact_id)
    assert dict(linked.links)["plan_id"] == plan.plan_id
    assert dict(linked.links)["invocation_id"] == harness.executor.active_invocation().invocation_id


def test_conformance_object_acceptance_requires_confirmation() -> None:
    harness = ConformanceHarness()
    harness.publish_arm()
    plan = harness.make_plan()
    harness.executor.execute_pose(plan_id=plan.plan_id, caller_id="agent-1")
    assert harness.run_to_terminal() is ActionStatus.SUCCEEDED

    criteria = evaluate_robot_state_criteria(*_home_targets(), [])
    result = evaluate_acceptance(
        invocation_status=ActionStatus.SUCCEEDED,
        criteria=criteria,
        profile=AcceptanceProfile.OPERATOR_CONFIRMED_OBJECT_STATE,
    )
    assert result.outcome.value == "inconclusive"


# ---------------------------------------------------------------------------
# Install verification and rollback (no robot)
# ---------------------------------------------------------------------------


def test_conformance_install_verification_tamper_rejection_and_rollback(
    tmp_path: Path,
) -> None:
    archive = package(BUNDLE, tmp_path)
    digest = sha256_file(archive)

    root = tmp_path / "skills"
    store = RuntimeStateStore(tmp_path / "state")
    installer = SkillInstaller(root=root, state_store=store)

    # Immutable verification: the packaged archive installs under its hash.
    manifest = installer.install(archive, expected_sha256=digest)
    assert manifest.name == "g1d-manipulation"
    assert load_manifest(root / "g1d-manipulation" / "skill.yaml").version == manifest.version

    # A tampered archive fails hash verification and installs nothing new.
    tampered = tmp_path / "tampered.tar.gz"
    shutil.copyfile(archive, tampered)
    with open(tampered, "ab") as handle:
        handle.write(b"tamper")
    with pytest.raises(Exception, match="sha256|hash|manifest"):
        installer.install(tampered, expected_sha256=digest)
    # The original installation is untouched (rollback left it intact).
    assert load_manifest(root / "g1d-manipulation" / "skill.yaml").version == manifest.version

    # Reinstall of the same archive keeps a backup of the previous version.
    installer.install(archive, expected_sha256=digest)
    backups = root / ".backups" / "g1d-manipulation"
    assert backups.is_dir() and any(backups.iterdir())

    # Remove cleans the install; a second remove is an explicit error.
    installer.remove("g1d-manipulation")
    assert not (root / "g1d-manipulation").exists()
    with pytest.raises(Exception, match="not installed"):
        installer.remove("g1d-manipulation")
    # The extracted bundle still passes archive validation offline.
    extracted = tmp_path / "extracted"
    ArchiveValidator().extract(archive, extracted, expected_sha256=digest)
    assert (extracted / "skill.yaml").is_file()
