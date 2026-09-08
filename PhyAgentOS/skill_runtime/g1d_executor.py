"""Execute and stop Actions for the visual-free G1_D dual-arm Skill.

``g1d.dual_arm.execute_pose`` turns one validated plan identity into an atomic
physical Action: admission, a current-state hold, one synchronized quintic
joint trajectory streamed as complete 35-entry command frames, watchdog
reconciliation, and a terminal lifecycle outcome.  ``g1d.dual_arm.stop``
idempotently stops the active invocation.  Everything here runs against the
robot-adapter seam: no SDK, DDS, or robot import is needed.
"""

from __future__ import annotations

import enum
import fcntl
import hashlib
import json
import sqlite3
import threading
import uuid
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass, replace
from functools import wraps
from pathlib import Path
from typing import Any

from PhyAgentOS.skill_runtime.g1d_adapter import (
    ARM_SLOTS,
    AdapterError,
    G1DAdapter,
    LowCmdFrame,
    StaleStateError,
)
from PhyAgentOS.skill_runtime.g1d_dex1 import (
    DEX1_SIDES,
    Dex1CommandSample,
    Dex1Error,
    Dex1Integration,
)
from PhyAgentOS.skill_runtime.g1d_planner import (
    G1DPlanner,
    PosePlan,
)
from PhyAgentOS.skill_runtime.g1d_trajectory import StopTrajectory, plan_stop, quintic_duration


class ExecutorError(RuntimeError):
    """Base class for explicit execute/stop Action failures."""


class BindingMismatchError(ExecutorError):
    """The plan was not produced by the current Skill/Runtime/profile binding."""


class ActiveActionError(ExecutorError):
    """Another physical Action is already active."""


class UnknownInvocationError(ExecutorError):
    """No invocation exists with the requested identity."""


class InvalidDeadlineError(ExecutorError):
    """The operation deadline is outside the admitted range."""


class Dex1GateError(ExecutorError):
    """A requested Dex1 side failed the readiness gate."""


class ActionStatus(enum.Enum):
    """Operation lifecycle of one physical Action invocation."""

    PENDING = "pending"
    RUNNING = "running"
    STOPPING = "stopping"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"
    STOPPED = "stopped"
    DEADLINE_EXCEEDED = "deadline_exceeded"
    UNKNOWN = "unknown"

    @property
    def is_terminal(self) -> bool:
        return self in _TERMINAL_STATUSES


_TERMINAL_STATUSES = frozenset(
    {
        ActionStatus.SUCCEEDED,
        ActionStatus.FAILED,
        ActionStatus.CANCELLED,
        ActionStatus.STOPPED,
        ActionStatus.DEADLINE_EXCEEDED,
        ActionStatus.UNKNOWN,
    }
)

DEFAULT_CONTROL_PERIOD_S = 0.002
DEFAULT_OPERATION_DEADLINE_S = 30.0
MAX_OPERATION_DEADLINE_S = 120.0
MIN_OPERATION_DEADLINE_S = 1.0
DEFAULT_HOLD_CYCLES = 10
DEFAULT_STOP_MARGIN_S = 0.2
DEFAULT_WATCHDOG_MISSED_CYCLES = 10


class StreamPhase(enum.Enum):
    """What one streamed frame is doing on the wire."""

    HOLD = "hold"
    MOTION = "motion"


class _ControlPhase(enum.Enum):
    """Internal writer state machine (not part of the public stream)."""

    HOLD = "hold"
    MOTION = "motion"
    POST_HOLD = "post_hold"
    STOP_HOLD = "stop_hold"


@dataclass(frozen=True)
class StreamSample:
    """One complete command frame as streamed to rt/lowcmd."""

    seq: int
    t_s: float
    frame: LowCmdFrame
    phase: StreamPhase


def _quintic(s: float) -> float:
    """Zero-velocity, zero-acceleration quintic blend from 0 to 1."""

    s = min(1.0, max(0.0, s))
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


@dataclass
class Invocation:
    """Public record of one execute_pose Action invocation."""

    invocation_id: str
    attempt_id: str
    plan_id: str
    caller_id: str
    status: ActionStatus
    created_at: float
    deadline_at: float
    stop_reason: str | None = None
    terminal_at: float | None = None
    frames_emitted: int = 0

    def as_result(self) -> dict[str, Any]:
        """Forge Tool result shape for execute_pose."""
        return {"invocation_id": self.invocation_id, "attempt_id": self.attempt_id}


def _serialized(method: Callable[..., Any]) -> Callable[..., Any]:
    @wraps(method)
    def call(self: Any, *args: Any, **kwargs: Any) -> Any:
        with self._lock:
            return method(self, *args, **kwargs)

    return call


class G1DExecutor:
    """Admit, stream, and reconcile exactly one physical dual-arm Action."""

    def __init__(
        self,
        *,
        adapter: G1DAdapter,
        planner: G1DPlanner,
        clock: Callable[[], float],
        sink: Any,
        skill_version: str,
        runtime_instance_id: str,
        profile_digest: str,
        control_period_s: float = DEFAULT_CONTROL_PERIOD_S,
        hold_cycles: int = DEFAULT_HOLD_CYCLES,
        stop_margin_s: float = DEFAULT_STOP_MARGIN_S,
        watchdog_missed_cycles: int = DEFAULT_WATCHDOG_MISSED_CYCLES,
        dex1: Dex1Integration | None = None,
        journal_path: Path | None = None,
    ) -> None:
        self._lock = threading.RLock()
        self._closed = False
        self._adapter = adapter
        self._dex1 = dex1
        self._planner = planner
        self._clock = clock
        self._sink = sink
        self._skill_version = skill_version
        self._runtime_instance_id = runtime_instance_id
        self._profile_digest = profile_digest
        self._control_period_s = float(control_period_s)
        self._hold_cycles = int(hold_cycles)
        self._stop_margin_s = float(stop_margin_s)
        self._watchdog_missed_cycles = int(watchdog_missed_cycles)
        self._invocations: dict[str, Invocation] = {}
        self._by_caller_plan: dict[tuple[str, str], str] = {}
        self._active: Invocation | None = None
        self._journal_lock = None
        if journal_path is not None:
            self._journal_lock = journal_path.with_suffix(journal_path.suffix + ".lock").open("a")
            try:
                fcntl.flock(self._journal_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                self._journal_lock.close()
                raise
        self._journal = sqlite3.connect(
            str(journal_path) if journal_path else ":memory:", check_same_thread=False
        )
        self._journal.execute("PRAGMA synchronous=FULL")
        self._journal.execute(
            "CREATE TABLE IF NOT EXISTS actions (id TEXT PRIMARY KEY, record TEXT NOT NULL)"
        )
        self._journal.execute(
            "CREATE TABLE IF NOT EXISTS evidence (invocation TEXT, phase TEXT, record TEXT, PRIMARY KEY(invocation, phase))"
        )
        self._stream_evidence: list[dict[str, Any]] = []
        self._pose_evidence: list[dict[str, Any]] = []
        for (raw,) in self._journal.execute("SELECT record FROM actions").fetchall():
            data = json.loads(raw)
            data["status"] = ActionStatus(data["status"])
            record = Invocation(**data)
            if not record.status.is_terminal:
                record.status = ActionStatus.UNKNOWN
                record.stop_reason = "runtime_restart"
                record.terminal_at = self._clock()
            self._invocations[record.invocation_id] = record
            self._by_caller_plan[(record.caller_id, record.plan_id)] = record.invocation_id
            self._persist(record)
        # Streaming state for the active invocation.
        self._phase: _ControlPhase | None = None
        self._seq = 0
        self._next_emit_t: float | None = None
        self._last_emit_t: float | None = None
        self._hold_until: float | None = None
        self._motion_start: float | None = None
        self._motion_duration_s: float | None = None
        self._post_hold_until: float | None = None
        self._stop_hold_until: float | None = None
        self._start_q: tuple[float, ...] = (0.0,) * 14
        self._target_q: tuple[float, ...] = (0.0,) * 14
        self._last_command: LowCmdFrame | None = None
        self._plan: PosePlan | None = None
        self._stop_trajectory: StopTrajectory | None = None
        self._stop_started_at = 0.0
        self._accepted_since: float | None = None
        self._observed_frame: int | None = None
        self._observed_at: float | None = None
        self._requested_dex1: dict[str, float | None] = {"left": None, "right": None}

    # ------------------------------------------------------------------
    # execute_pose admission
    # ------------------------------------------------------------------

    @_serialized
    def execute_pose(
        self,
        *,
        plan_id: str,
        caller_id: str,
        operation_deadline_s: float = DEFAULT_OPERATION_DEADLINE_S,
        invocation_id: str | None = None,
        attempt_id: str | None = None,
    ) -> Invocation:
        """Admit one current plan as a physical Action (idempotent by caller+plan)."""

        if self._closed:
            raise ExecutorError("executor is closed")
        existing_id = self._by_caller_plan.get((caller_id, plan_id))
        if existing_id is not None:
            return self._invocations[existing_id]

        if any(record.status is ActionStatus.UNKNOWN for record in self._invocations.values()):
            raise ExecutorError("unresolved physical outcome requires operator recovery")
        if self._active is not None and not self._active.status.is_terminal:
            raise ActiveActionError(f"invocation {self._active.invocation_id!r} is still active")

        if not MIN_OPERATION_DEADLINE_S <= operation_deadline_s <= MAX_OPERATION_DEADLINE_S:
            raise InvalidDeadlineError(
                f"operation_deadline_s must be within [{MIN_OPERATION_DEADLINE_S},"
                f" {MAX_OPERATION_DEADLINE_S}]"
            )

        plan = self._require_current_plan(plan_id)
        self._require_binding(plan)
        self._plan = plan
        # Readiness gate: fresh CRC-valid state, approved mode, safety healthy.
        state = self._adapter.require_ready()
        current = tuple(m.q for m in (*state.left_arm, *state.right_arm))
        if plan.start_q is None:
            raise ExecutorError("execution requires validated planned start state")
        if not plan.checks_passed or any(
            abs(a - b) > 0.02 for a, b in zip(current, plan.start_q, strict=True)
        ):
            raise ExecutorError("plan checks failed or current state differs from planned start")
        # Optional external Dex1: only *requested* openings are gated; a
        # missing opening is non-blocking and the service may be absent.
        if plan.dex1_left_opening is not None or plan.dex1_right_opening is not None:
            if self._dex1 is None:
                raise ExecutorError(
                    "plan requests a Dex1 opening but no Dex1 integration is configured"
                )
            try:
                self._dex1.require_ready_for(
                    {"left": plan.dex1_left_opening, "right": plan.dex1_right_opening}
                )
            except Dex1Error as error:
                raise Dex1GateError(str(error)) from error

        now = self._clock()
        invocation = Invocation(
            invocation_id=invocation_id or f"inv-{uuid.uuid4().hex}",
            attempt_id=attempt_id or f"attempt-{uuid.uuid4().hex}",
            plan_id=plan_id,
            caller_id=caller_id,
            status=ActionStatus.PENDING,
            created_at=now,
            deadline_at=now + float(operation_deadline_s),
        )
        if invocation.invocation_id in self._invocations:
            raise ExecutorError("invocation identity is already bound to another operation")
        with self._journal:
            self._write_evidence(
                invocation, "before", {"state": asdict(state), "plan": asdict(plan)}
            )
            self._persist(invocation)  # Commit admission and before evidence before any effect.
        self._stream_evidence = []
        self._pose_evidence = []
        self._invocations[invocation.invocation_id] = invocation
        self._requested_dex1 = {
            "left": plan.dex1_left_opening,
            "right": plan.dex1_right_opening,
        }
        self._seq = 0
        self._next_emit_t = None
        self._last_emit_t = None
        self._last_command = None
        self._phase = None
        self._accepted_since = None
        self._observed_frame = None
        self._observed_at = None
        self._by_caller_plan[(caller_id, plan_id)] = invocation.invocation_id
        self._active = invocation
        self._adapter.set_active_operation(invocation.invocation_id)
        return invocation

    def _persist(self, invocation: Invocation) -> None:
        data = asdict(invocation)
        data["status"] = invocation.status.value
        with self._journal:
            self._journal.execute(
                "INSERT OR REPLACE INTO actions VALUES (?, ?)",
                (invocation.invocation_id, json.dumps(data, allow_nan=False)),
            )

    def _write_evidence(self, invocation: Invocation, phase: str, payload: dict[str, Any]) -> None:
        record = {
            "phase": phase,
            "invocation_id": invocation.invocation_id,
            "attempt_id": invocation.attempt_id,
            "plan_id": invocation.plan_id,
            "runtime_instance_id": self._runtime_instance_id,
            "skill_version": self._skill_version,
            "profile_digest": self._profile_digest,
            "monotonic_at": self._clock(),
            "payload": payload,
        }
        raw = json.dumps(record, sort_keys=True, allow_nan=False)
        record["sha256"] = hashlib.sha256(raw.encode()).hexdigest()
        self._journal.execute(
            "INSERT OR REPLACE INTO evidence VALUES (?, ?, ?)",
            (invocation.invocation_id, phase, json.dumps(record, allow_nan=False)),
        )

    @_serialized
    def evidence(self, invocation_id: str) -> list[dict[str, Any]]:
        self.get_invocation(invocation_id)
        records = {
            phase: json.loads(raw)
            for phase, raw in self._journal.execute(
                "SELECT phase, record FROM evidence WHERE invocation=?", (invocation_id,)
            )
        }
        return [records[phase] for phase in ("before", "during", "after") if phase in records]

    @_serialized
    def close(self) -> None:
        if self._closed:
            return
        if self._active is not None and not self._active.status.is_terminal:
            self.mark_unknown(self._active.invocation_id, reason="runtime_shutdown")
        self._journal.close()
        if self._journal_lock is not None:
            self._journal_lock.close()
        self._closed = True

    def _require_current_plan(self, plan_id: str) -> PosePlan:
        # Raises PlanExpiredError for unknown or expired plans.
        return self._planner.get_plan(plan_id)

    @_serialized
    def while_idle(self, operation: Callable[[], Any]) -> Any:
        """Serialize shared-model planning against Action admission."""
        if self._active is not None and not self._active.status.is_terminal:
            raise ExecutorError("planning requires an idle executor")
        return operation()

    def _require_binding(self, plan: PosePlan) -> None:
        binding = plan.binding
        mismatches = []
        if binding.skill_version != self._skill_version:
            mismatches.append(f"skill_version {binding.skill_version!r} != {self._skill_version!r}")
        if binding.runtime_instance_id != self._runtime_instance_id:
            mismatches.append(
                f"runtime_instance_id {binding.runtime_instance_id!r} != {self._runtime_instance_id!r}"
            )
        if binding.profile_digest != self._profile_digest:
            mismatches.append("profile_digest differs from the active profile")
        if mismatches:
            raise BindingMismatchError(
                "plan binding does not match the active Skill/Runtime: " + "; ".join(mismatches)
            )

    @_serialized
    def get_invocation(self, invocation_id: str) -> Invocation:
        try:
            return self._invocations[invocation_id]
        except KeyError:
            raise UnknownInvocationError(f"unknown invocation {invocation_id!r}") from None

    # ------------------------------------------------------------------
    # stop Action
    # ------------------------------------------------------------------

    @_serialized
    def stop(self, *, invocation_id: str | None = None) -> str:
        """Idempotently request a controlled stop; targets the active invocation."""

        if invocation_id is not None:
            record = self._invocations.get(invocation_id)
            if record is None or record.status is ActionStatus.UNKNOWN:
                return "unknown"
        active = self._active
        if invocation_id is None and any(
            record.status is ActionStatus.UNKNOWN for record in self._invocations.values()
        ):
            return "unknown"
        if active is not None and active.status is ActionStatus.UNKNOWN:
            return "unknown"
        if active is None or active.status.is_terminal:
            return "already_stopped"
        if invocation_id is not None and invocation_id != active.invocation_id:
            return "already_stopped"
        if active.status is ActionStatus.STOPPING:
            return "accepted"
        self._request_stop(active)
        return "accepted"

    # ------------------------------------------------------------------
    # lifecycle observation and reconciliation
    # ------------------------------------------------------------------

    @_serialized
    def get_active_status(self) -> ActionStatus | None:
        return self._active.status if self._active is not None else None

    @_serialized
    def active_invocation(self) -> Invocation | None:
        return self._active

    @_serialized
    def query_state(self) -> dict[str, Any]:
        """g1d.dual_arm.state output: runtime readiness, action readiness, Dex1."""

        try:
            adapter_state = self._adapter.query_state()
            runtime_ready = True
            state_age_ms: float | None = adapter_state.state_age_ms
            safety_gate: str = adapter_state.safety_gate
        except AdapterError as error:
            runtime_ready = False
            state_age_ms = None
            safety_gate = type(error).__name__
        try:
            self._adapter.require_ready()
            gate_ready = True
        except AdapterError:
            gate_ready = False
        action_ready = (
            gate_ready
            and not self._closed
            and not any(
                record.status is ActionStatus.UNKNOWN for record in self._invocations.values()
            )
            and not (self._active is not None and not self._active.status.is_terminal)
        )
        dex1: dict[str, dict[str, Any]] = {}
        for side in DEX1_SIDES:
            if self._dex1 is None:
                dex1[side] = {"status": "absent", "opening": None, "age_ms": None}
                continue
            readiness = self._dex1.readiness(side)
            dex1[side] = {
                "status": readiness.status.value,
                "opening": readiness.opening,
                "age_ms": readiness.age_ms,
            }
        return {
            "runtime_ready": runtime_ready,
            "action_ready": action_ready,
            "state_age_ms": state_age_ms,
            "safety_gate": safety_gate,
            "dex1": dex1,
        }

    @_serialized
    def mark_unknown(self, invocation_id: str, *, reason: str) -> None:
        """Record an unresolved physical outcome (transport loss, crash)."""

        invocation = self.get_invocation(invocation_id)
        if invocation.status.is_terminal:
            raise ExecutorError(
                f"invocation {invocation_id!r} is already {invocation.status.value}"
            )
        self._finish(invocation, ActionStatus.UNKNOWN, reason=reason)

    # ------------------------------------------------------------------
    # 500 Hz writer loop
    # ------------------------------------------------------------------

    @_serialized
    def tick(self) -> None:
        """Advance the writer by one control cycle (called at ~2 ms cadence)."""

        invocation = self._active
        if invocation is None or invocation.status.is_terminal:
            return
        now = self._clock()

        if invocation.status is ActionStatus.STOPPING:
            self._tick_stop_hold(invocation, now)
            return

        # Safety gate: fresh CRC-valid state, approved mode, no safety fault.
        try:
            self._adapter.require_ready()
        except StaleStateError:
            # Writer watchdog: stale state stops admission/execution.
            self._watchdog_stop(invocation, now, reason="stale_state")
            return
        except AdapterError as error:
            self._finish(invocation, ActionStatus.UNKNOWN, reason=type(error).__name__)
            return

        # Requested Dex1 streams must stay healthy for the whole Action.
        if self._dex1 is not None and any(
            opening is not None for opening in self._requested_dex1.values()
        ):
            for side, opening in self._requested_dex1.items():
                if opening is None:
                    continue
                readiness = self._dex1.readiness(side)
                if not readiness.ok:
                    self._watchdog_stop(
                        invocation, now, reason=f"dex1_{side}_{readiness.status.value}"
                    )
                    return

        # Writer watchdog: ten missed 2 ms command cycles.
        if (
            self._last_emit_t is not None
            and now - self._last_emit_t > self._watchdog_missed_cycles * self._control_period_s
        ):
            self._watchdog_stop(invocation, now, reason="watchdog_missed_cycles")
            return

        # Operation deadline: stop handling inside the reserved margin.
        if now >= invocation.deadline_at - 2.0 - self._stop_margin_s - 0.1:
            if self._seq == 0:
                self._finish(
                    invocation, ActionStatus.DEADLINE_EXCEEDED, reason="deadline_before_effect"
                )
                return
            self._begin_stop(invocation, now, reason="deadline")
            return

        self._stream(invocation, now)

    # ------------------------------------------------------------------
    # streaming internals
    # ------------------------------------------------------------------

    def _stream(self, invocation: Invocation, now: float) -> None:
        if self._next_emit_t is None:
            # First writer cycle: schedule from now and enter the hold phase.
            self._next_emit_t = now
            self._hold_until = now + self._hold_cycles * self._control_period_s
            self._phase = _ControlPhase.HOLD
            self._arm_motion(invocation)
            if invocation.status.is_terminal:
                return

        if self._last_emit_t is None or now > self._last_emit_t:
            emit_t = now
            if self._phase is _ControlPhase.HOLD:
                hold_until = self._hold_until
                assert hold_until is not None
                if invocation.status is ActionStatus.PENDING:
                    invocation.status = ActionStatus.RUNNING
                self._emit(emit_t, self._adapter.hold_frame(), phase=StreamPhase.HOLD)
                if emit_t >= hold_until:
                    self._phase = _ControlPhase.MOTION
                    self._motion_start = emit_t + self._control_period_s
            elif self._phase is _ControlPhase.MOTION:
                motion_start = self._motion_start
                motion_duration = self._motion_duration_s
                assert motion_start is not None and motion_duration is not None
                self._emit(emit_t, self._motion_frame(emit_t), phase=StreamPhase.MOTION)
                motion_end = motion_start + motion_duration
                if emit_t >= motion_end - self._control_period_s / 2:
                    self._phase = _ControlPhase.POST_HOLD
                    self._post_hold_until = emit_t + self._stop_margin_s
            elif self._phase is _ControlPhase.POST_HOLD:
                post_hold_until = self._post_hold_until
                assert post_hold_until is not None
                self._emit(emit_t, self._motion_frame(emit_t), phase=StreamPhase.HOLD)
                if self._observed_acceptance(emit_t):
                    self._finish(invocation, ActionStatus.SUCCEEDED, reason=None)
                    return
            self._emit_dex1(emit_t)
            self._next_emit_t = emit_t + self._control_period_s

    def _observed_acceptance(self, now: float) -> bool:
        state = self._adapter.require_ready()
        if self._observed_at is not None and now - self._observed_at > 0.1:
            self._accepted_since = None
        if state.frame == self._observed_frame:
            return False
        self._observed_frame = state.frame
        self._observed_at = now
        q = [m.q for m in (*state.left_arm, *state.right_arm)]
        assert self._plan is not None
        try:
            observation = self._planner.observe_pose(self._plan, q)
            observation["state_age_ms"] = state.state_age_ms
            observation["safety_gate"] = "ready"
            observation["dex1"] = {}
            self._pose_evidence.append(
                {"monotonic_at": now, "state_frame": state.frame, **observation}
            )
            within = observation["within_tolerance"]
        except (ValueError, RuntimeError):
            within = False
        if self._dex1 is not None:
            for side, target in self._requested_dex1.items():
                if target is not None:
                    observed = self._dex1.readiness(side)
                    if self._pose_evidence:
                        self._pose_evidence[-1]["dex1"][side] = {
                            "target": target,
                            "opening": observed.opening,
                            "healthy": observed.ok,
                            "age_ms": observed.age_ms,
                        }
                    within = (
                        within
                        and observed.ok
                        and observed.opening is not None
                        and abs(observed.opening - target) <= 0.05
                    )
        if not within:
            self._accepted_since = None
            return False
        if self._accepted_since is None:
            self._accepted_since = now
        return now - self._accepted_since >= 2.0

    def _emit_dex1(self, t: float) -> None:
        """Stream requested opening commands to the external Dex1 topics."""

        if self._dex1 is None:
            return
        for side, opening in self._requested_dex1.items():
            if opening is None:
                continue
            self._write_command(Dex1CommandSample(t_s=t, command=self._dex1.command(side, opening)))

    def _arm_motion(self, invocation: Invocation) -> None:
        plan = self._plan
        assert plan is not None
        state = self._adapter.query_state()
        current = [snapshot.q for snapshot in (*state.left_arm, *state.right_arm)]
        if plan.start_q is None or any(
            abs(a - b) > 0.02 for a, b in zip(current, plan.start_q, strict=True)
        ):
            self._finish(
                invocation, ActionStatus.FAILED, reason="start_state_changed_before_effect"
            )
            return
        target = [*plan.joint_solution.left_q, *plan.joint_solution.right_q]
        self._start_q = tuple(current)
        self._target_q = tuple(target)
        self._motion_duration_s = quintic_duration(
            current,
            target,
            minimum_duration_s=self._planner.minimum_duration_s,
            max_velocity=self._planner.max_joint_velocity_rad_per_s,
            max_acceleration=self._planner.max_joint_acceleration_rad_per_s2,
            max_jerk=self._planner.max_joint_jerk_rad_per_s3,
        )

    def _motion_frame(self, t: float) -> LowCmdFrame:
        assert self._motion_start is not None and self._motion_duration_s is not None
        s = min(1.0, max(0.0, (t - self._motion_start) / self._motion_duration_s))
        blend = _quintic(s)
        q = tuple(
            start + (target - start) * blend
            for start, target in zip(self._start_q, self._target_q, strict=True)
        )
        dq = tuple(
            (b - a) * 30 * s * s * (1 - s) * (1 - s) / self._motion_duration_s
            for a, b in zip(self._start_q, self._target_q, strict=True)
        )
        return self._overlay_arm_q(q, dq)

    def _overlay_arm_q(
        self, arm_q: Sequence[float], arm_dq: Sequence[float] | None = None
    ) -> LowCmdFrame:
        """Rebuild the last complete frame with new arm-slot commands (15-28)."""

        template = self._last_command
        if template is None:
            template = self._adapter.hold_frame()
        commands = list(template.motor_cmd)
        for index, value in enumerate(arm_q):
            commands[ARM_SLOTS[index]] = replace(
                commands[ARM_SLOTS[index]],
                q=float(value),
                dq=float(arm_dq[index]) if arm_dq is not None else 0.0,
            )
        return LowCmdFrame(
            mode_machine=template.mode_machine,
            motor_cmd=tuple(commands),
            mode_pr=template.mode_pr,
            reserve=template.reserve,
        ).with_crc()

    def _write_command(self, sample: Any) -> None:
        try:
            self._sink.write(sample)
        except Exception:
            if self._active is not None:
                self._finish(self._active, ActionStatus.UNKNOWN, reason="command_write_error")
            raise

    def _emit(self, t: float, frame: LowCmdFrame, *, phase: StreamPhase) -> None:
        seq = self._seq
        self._seq = seq + 1
        self._last_emit_t = t
        self._last_command = frame
        self._write_command(StreamSample(seq=seq, t_s=t, frame=frame, phase=phase))
        self._stream_evidence.append(
            {"seq": seq, "monotonic_at": t, "phase": phase.value, "frame": asdict(frame)}
        )
        if self._active is not None:
            self._active.frames_emitted = self._seq

    def _finish(self, invocation: Invocation, status: ActionStatus, *, reason: str | None) -> None:
        """Terminate an invocation and release the active-operation marker."""

        self._terminate(invocation, status, reason=reason)
        self._adapter.set_active_operation(None)

    # ------------------------------------------------------------------
    # stop handling
    # ------------------------------------------------------------------

    def _request_stop(self, invocation: Invocation) -> None:
        now = self._clock()
        if self._seq == 0:
            # Pre-effect cancellation: nothing has streamed yet.
            self._finish(invocation, ActionStatus.CANCELLED, reason="operator_stop")
            return
        self._begin_stop(invocation, now, reason="operator_stop")

    def _begin_stop(self, invocation: Invocation, now: float, *, reason: str) -> None:
        """Enter stopping: bounded hold within the stop margin, then terminal."""

        assert self._last_command is not None
        q = [self._last_command.motor_cmd[i].q for i in ARM_SLOTS]
        dq = [self._last_command.motor_cmd[i].dq for i in ARM_SLOTS]
        ddq = [0.0] * 14
        if (
            self._phase is _ControlPhase.MOTION
            and self._motion_start is not None
            and self._motion_duration_s is not None
        ):
            u = min(
                1.0,
                max(
                    0.0, ((self._last_emit_t or now) - self._motion_start) / self._motion_duration_s
                ),
            )
            ddq = [
                (b - a) * 60 * u * (1 - u) * (1 - 2 * u) / self._motion_duration_s**2
                for a, b in zip(self._start_q, self._target_q, strict=True)
            ]
        try:
            self._stop_trajectory = plan_stop(
                q,
                dq,
                ddq,
                limits=self._planner.joint_limits,
                velocity=self._planner.max_joint_velocity_rad_per_s,
                acceleration=self._planner.max_joint_acceleration_rad_per_s2,
                jerk=self._planner.max_joint_jerk_rad_per_s3,
                budget_s=2.0,
            )
        except ValueError:
            self._finish(invocation, ActionStatus.UNKNOWN, reason="bounded_stop_unavailable")
            return
        self._stop_started_at = now
        invocation.status = ActionStatus.STOPPING
        invocation.stop_reason = reason
        base = self._last_emit_t if self._last_emit_t is not None else now
        self._stop_hold_until = (
            max(base, now) + self._stop_trajectory.duration_s + self._stop_margin_s + 0.1
        )
        self._accepted_since = None
        self._observed_at = None
        self._observed_frame = None
        self._phase = _ControlPhase.STOP_HOLD

    def _tick_stop_hold(self, invocation: Invocation, now: float) -> None:
        try:
            state = self._adapter.require_ready()
            if self._dex1 is not None:
                self._dex1.require_ready_for(self._requested_dex1)
        except (AdapterError, Dex1Error) as error:
            self._finish(invocation, ActionStatus.UNKNOWN, reason=type(error).__name__)
            return
        if (
            self._last_emit_t is not None
            and now - self._last_emit_t > self._watchdog_missed_cycles * self._control_period_s
        ):
            self._finish(invocation, ActionStatus.UNKNOWN, reason="stop_writer_gap")
            return
        assert self._last_command is not None
        if self._last_emit_t is None or now > self._last_emit_t:
            assert self._stop_trajectory is not None
            q, dq = self._stop_trajectory.sample(now - self._stop_started_at)
            self._emit(now, self._overlay_arm_q(q, dq), phase=StreamPhase.HOLD)
            self._next_emit_t = now + self._control_period_s
        assert self._stop_trajectory is not None
        settled = now - self._stop_started_at >= self._stop_trajectory.duration_s
        within = settled and all(
            abs(m.q - self._last_command.motor_cmd[m.slot].q) <= 0.01
            for m in (*state.left_arm, *state.right_arm)
        )
        if self._observed_at is not None and now - self._observed_at > 0.1:
            self._accepted_since = None
        if state.frame != self._observed_frame:
            self._observed_frame = state.frame
            self._observed_at = now
            if not within:
                self._accepted_since = None
            elif self._accepted_since is None:
                self._accepted_since = now
            elif now - self._accepted_since >= self._stop_margin_s:
                terminal = (
                    ActionStatus.DEADLINE_EXCEEDED
                    if invocation.stop_reason == "deadline"
                    else ActionStatus.STOPPED
                )
                self._finish(invocation, terminal, reason=invocation.stop_reason)
                return
        if now >= (self._stop_hold_until or now):
            self._finish(invocation, ActionStatus.UNKNOWN, reason="hold_unconfirmed")

    def _watchdog_stop(self, invocation: Invocation, now: float, *, reason: str) -> None:
        """Unhealthy feedback/cadence cannot establish a physical stop."""
        self._finish(invocation, ActionStatus.UNKNOWN, reason=reason)

    def _terminate(
        self, invocation: Invocation, status: ActionStatus, *, reason: str | None
    ) -> None:
        candidate = replace(
            invocation, status=status, stop_reason=reason, terminal_at=self._clock()
        )
        try:
            with self._journal:
                self._write_evidence(
                    candidate,
                    "during",
                    {"frames": self._stream_evidence, "observations": self._pose_evidence},
                )
                self._write_evidence(
                    candidate,
                    "after",
                    {"status": status.value, "reason": reason, "state": self.query_state()},
                )
                self._persist(candidate)
        except Exception:
            invocation.status = ActionStatus.UNKNOWN
            invocation.stop_reason = "terminal_evidence_unavailable"
            invocation.terminal_at = self._clock()
            self._adapter.set_active_operation(None)
            raise
        invocation.status = candidate.status
        invocation.stop_reason = candidate.stop_reason
        invocation.terminal_at = candidate.terminal_at


class RecordingSink:
    """Command sink that records every streamed frame for tests and audits."""

    def __init__(self) -> None:
        self.samples: list[Any] = []

    def write(self, sample: Any) -> None:
        self.samples.append(sample)


__all__ = [
    "ActionStatus",
    "ActiveActionError",
    "BindingMismatchError",
    "ExecutorError",
    "G1DExecutor",
    "Invocation",
    "RecordingSink",
    "StreamPhase",
    "StreamSample",
    "UnknownInvocationError",
]
