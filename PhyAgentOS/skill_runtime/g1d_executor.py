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
import math
import uuid
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field, replace
from typing import Any

from PhyAgentOS.skill_runtime.g1d_adapter import (
    ARM_SLOTS,
    G1DAdapter,
    LowCmdFrame,
    MotorCommand,
    ModeLossError,
    SafetyFaultError,
    StaleStateError,
    TOTAL_MOTOR_SLOTS,
)
from PhyAgentOS.skill_runtime.g1d_planner import (
    G1DPlanner,
    PlanExpiredError,
    PosePlan,
)


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


def _quintic_duration_s(
    start: Sequence[float],
    target: Sequence[float],
    *,
    minimum_duration_s: float,
    max_velocity: float,
    max_acceleration: float,
) -> float:
    """Duration so the quintic profile respects velocity/acceleration caps."""

    max_delta = max((abs(b - a) for a, b in zip(start, target, strict=True)), default=0.0)
    duration = minimum_duration_s
    if max_delta > 0:
        duration = max(
            duration,
            (15.0 / 8.0) * max_delta / max_velocity,
            math.sqrt((10.0 * math.sqrt(3.0) / 3.0) * max_delta / max_acceleration),
        )
    return duration


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
    ) -> None:
        self._adapter = adapter
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

    # ------------------------------------------------------------------
    # execute_pose admission
    # ------------------------------------------------------------------

    def execute_pose(
        self,
        *,
        plan_id: str,
        caller_id: str,
        operation_deadline_s: float = DEFAULT_OPERATION_DEADLINE_S,
    ) -> Invocation:
        """Admit one current plan as a physical Action (idempotent by caller+plan)."""

        existing_id = self._by_caller_plan.get((caller_id, plan_id))
        if existing_id is not None:
            return self._invocations[existing_id]

        if self._active is not None and not self._active.status.is_terminal:
            raise ActiveActionError(
                f"invocation {self._active.invocation_id!r} is still active"
            )

        if not MIN_OPERATION_DEADLINE_S <= operation_deadline_s <= MAX_OPERATION_DEADLINE_S:
            raise InvalidDeadlineError(
                f"operation_deadline_s must be within [{MIN_OPERATION_DEADLINE_S},"
                f" {MAX_OPERATION_DEADLINE_S}]"
            )

        plan = self._require_current_plan(plan_id)
        self._require_binding(plan)
        self._plan = plan
        # Readiness gate: fresh CRC-valid state, approved mode, safety healthy.
        self._adapter.require_ready()

        now = self._clock()
        invocation = Invocation(
            invocation_id=f"inv-{uuid.uuid4().hex}",
            attempt_id=f"attempt-{uuid.uuid4().hex}",
            plan_id=plan_id,
            caller_id=caller_id,
            status=ActionStatus.PENDING,
            created_at=now,
            deadline_at=now + float(operation_deadline_s),
        )
        self._invocations[invocation.invocation_id] = invocation
        self._seq = 0
        self._by_caller_plan[(caller_id, plan_id)] = invocation.invocation_id
        self._active = invocation
        self._adapter.set_active_operation(invocation.invocation_id)
        return invocation

    def _require_current_plan(self, plan_id: str) -> PosePlan:
        # Raises PlanExpiredError for unknown or expired plans.
        return self._planner.get_plan(plan_id)

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

    def get_invocation(self, invocation_id: str) -> Invocation:
        try:
            return self._invocations[invocation_id]
        except KeyError:
            raise UnknownInvocationError(f"unknown invocation {invocation_id!r}") from None

    # ------------------------------------------------------------------
    # stop Action
    # ------------------------------------------------------------------

    def stop(self, *, invocation_id: str | None = None) -> str:
        """Idempotently request a controlled stop; targets the active invocation."""

        if invocation_id is not None and invocation_id not in self._invocations:
            return "unknown"
        active = self._active
        if active is None or active.status.is_terminal:
            return "already_stopped"
        if invocation_id is not None and invocation_id != active.invocation_id:
            return "already_stopped"
        self._request_stop(active)
        return "accepted"

    # ------------------------------------------------------------------
    # lifecycle observation and reconciliation
    # ------------------------------------------------------------------

    def get_active_status(self) -> ActionStatus | None:
        return self._active.status if self._active is not None else None

    def active_invocation(self) -> Invocation | None:
        return self._active

    def mark_unknown(self, invocation_id: str, *, reason: str) -> None:
        """Record an unresolved physical outcome (transport loss, crash)."""

        invocation = self.get_invocation(invocation_id)
        if invocation.status.is_terminal:
            raise ExecutorError(
                f"invocation {invocation_id!r} is already {invocation.status.value}"
            )
        self._terminate(invocation, ActionStatus.UNKNOWN, reason=reason)

    # ------------------------------------------------------------------
    # 500 Hz writer loop
    # ------------------------------------------------------------------

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
        except (ModeLossError, SafetyFaultError) as error:
            # Urgent safety faults take the conservative stop path: stop
            # streaming immediately, no smooth deceleration wait.
            # Best-effort conservative stop command before giving up the wire.
            if self._last_command is not None:
                self._emit(now, self._last_command, phase=StreamPhase.HOLD)
            self._finish(invocation, ActionStatus.FAILED, reason=type(error).__name__)
            return

        # Writer watchdog: ten missed 2 ms command cycles.
        if (
            self._last_emit_t is not None
            and now - self._last_emit_t > self._watchdog_missed_cycles * self._control_period_s
        ):
            self._watchdog_stop(invocation, now, reason="watchdog_missed_cycles")
            return

        # Operation deadline: stop handling inside the reserved margin.
        if now >= invocation.deadline_at:
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

        while self._next_emit_t is not None and now >= self._next_emit_t:
            emit_t = self._next_emit_t
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
                if emit_t >= post_hold_until:
                    self._finish(invocation, ActionStatus.SUCCEEDED, reason=None)
                    return
            self._next_emit_t = emit_t + self._control_period_s

    def _arm_motion(self, invocation: Invocation) -> None:
        plan = self._plan
        state = self._adapter.query_state()
        current = [snapshot.q for snapshot in (*state.left_arm, *state.right_arm)]
        target = [*plan.joint_solution.left_q, *plan.joint_solution.right_q]
        self._start_q = tuple(current)
        self._target_q = tuple(target)
        self._motion_duration_s = _quintic_duration_s(
            current,
            target,
            minimum_duration_s=self._planner.minimum_duration_s,
            max_velocity=self._planner.max_joint_velocity_rad_per_s,
            max_acceleration=self._planner.max_joint_acceleration_rad_per_s2,
        )

    def _motion_frame(self, t: float) -> LowCmdFrame:
        assert self._motion_start is not None and self._motion_duration_s is not None
        s = (t - self._motion_start) / self._motion_duration_s
        blend = _quintic(s)
        q = tuple(
            start + (target - start) * blend
            for start, target in zip(self._start_q, self._target_q, strict=True)
        )
        return self._overlay_arm_q(q)

    def _overlay_arm_q(self, arm_q: Sequence[float]) -> LowCmdFrame:
        """Rebuild the last complete frame with new arm-slot commands (15-28)."""

        template = self._last_command
        if template is None:
            template = self._adapter.hold_frame()
        commands = list(template.motor_cmd)
        for index, value in enumerate(arm_q):
            commands[ARM_SLOTS[index]] = replace(commands[ARM_SLOTS[index]], q=float(value))
        return LowCmdFrame(
            mode_machine=template.mode_machine,
            motor_cmd=tuple(commands),
            mode_pr=template.mode_pr,
            reserve=template.reserve,
        ).with_crc()

    def _emit(self, t: float, frame: LowCmdFrame, *, phase: StreamPhase) -> None:
        seq = self._seq
        self._seq = seq + 1
        self._last_emit_t = t
        self._last_command = frame
        self._sink.write(StreamSample(seq=seq, t_s=t, frame=frame, phase=phase))
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

        invocation.status = ActionStatus.STOPPING
        invocation.stop_reason = reason
        base = self._last_emit_t if self._last_emit_t is not None else now
        self._stop_hold_until = max(base, now) + self._stop_margin_s
        self._phase = _ControlPhase.STOP_HOLD

    def _tick_stop_hold(self, invocation: Invocation, now: float) -> None:
        # Hold at the last commanded state; the freshness gate is not
        # re-evaluated while stopping (a stale stream must not block the stop).
        # If the stop path itself overruns its margin plus the watchdog window,
        # reconcile to the terminal outcome instead of stopping forever.
        overrun_after = (self._stop_hold_until or now) + (
            self._watchdog_missed_cycles * self._control_period_s
        )
        if now > overrun_after:
            terminal = {
                "operator_stop": ActionStatus.STOPPED,
                "deadline": ActionStatus.DEADLINE_EXCEEDED,
            }.get(invocation.stop_reason or "", ActionStatus.STOPPED)
            self._finish(invocation, terminal, reason=invocation.stop_reason)
            return
        while self._next_emit_t is not None and now >= self._next_emit_t:
            emit_t = self._next_emit_t
            assert self._last_command is not None
            self._emit(emit_t, self._last_command, phase=StreamPhase.HOLD)
            self._next_emit_t = emit_t + self._control_period_s
            if emit_t >= (self._stop_hold_until or 0.0):
                terminal = {
                    "operator_stop": ActionStatus.STOPPED,
                    "deadline": ActionStatus.DEADLINE_EXCEEDED,
                }.get(invocation.stop_reason or "", ActionStatus.STOPPED)
                self._finish(invocation, terminal, reason=invocation.stop_reason)
                return

    def _watchdog_stop(self, invocation: Invocation, now: float, *, reason: str) -> None:
        """Declared watchdog stop path: stop streaming and account immediately.

        The writer loop is already unhealthy (missed cycles or stale state),
        so there is no reliable cadence left for a smooth stop-margin hold;
        one best-effort hold command is emitted and the invocation terminates
        as stopped so reconciliation can proceed.
        """

        if self._last_command is not None:
            self._emit(now, self._last_command, phase=StreamPhase.HOLD)
        self._finish(invocation, ActionStatus.STOPPED, reason=reason)

    def _terminate(self, invocation: Invocation, status: ActionStatus, *, reason: str | None) -> None:
        invocation.status = status
        invocation.stop_reason = reason
        invocation.terminal_at = self._clock()


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
