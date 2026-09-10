"""Production execution composition with an explicit robot-supervisor authority seam.

The site supervisor supplies verified gains/modes and a live ownership check.
There is intentionally no default approval or implicit MotionSwitcher restore.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import replace
from pathlib import Path
from typing import Any

from PhyAgentOS.skill_runtime.g1d_adapter import (
    G1DAdapter,
    LowCmdFrame,
    LowStateSource,
    SafetyFaultError,
)
from PhyAgentOS.skill_runtime.g1d_control_loop import G1DControlLoop
from PhyAgentOS.skill_runtime.g1d_dex1 import Dex1Integration
from PhyAgentOS.skill_runtime.g1d_executor import G1DExecutor
from PhyAgentOS.skill_runtime.g1d_planner import G1DPlanner, PlannerError, digest_json
from PhyAgentOS.skill_runtime.g1d_runtime import G1DReadOnlyRuntime


class _ApprovedAdapter(G1DAdapter):
    def __init__(
        self,
        *,
        clock: Callable[[], float],
        approved_mode: int,
        gains: Sequence[tuple[float, float]],
        modes: Sequence[int],
        require_ownership: Callable[[], None],
    ) -> None:
        super().__init__(clock=clock, approved_mode_machine=approved_mode)
        self.gains = tuple(gains)
        self.modes = tuple(modes)
        self.require_ownership = require_ownership

    def require_ready(self) -> Any:
        self.require_ownership()
        return super().require_ready()

    def hold_frame(self) -> LowCmdFrame:
        frame = super().hold_frame()
        return replace(
            frame,
            motor_cmd=tuple(
                replace(command, mode=mode, kp=kp, kd=kd)
                for command, (kp, kd), mode in zip(
                    frame.motor_cmd, self.gains, self.modes, strict=True
                )
            ),
        ).with_crc()


class G1DExecutionRuntime(G1DReadOnlyRuntime):
    """Supervised arm and gripper control with durable execution and camera queries.

    require_ownership must raise AdapterError whenever the approved handoff,
    exclusive lowcmd lease, operator safety gate, or verified limits lapse.
    Checking a local journal lock alone does not satisfy this contract.
    """

    def __init__(
        self,
        bundle_root: Path,
        *,
        source: LowStateSource,
        sink: Any,
        journal_path: Path,
        approved_mode: int,
        gains: Sequence[tuple[float, float]],
        modes: Sequence[int],
        require_ownership: Callable[[], None],
        clock: Callable[[], float],
        dex1: Dex1Integration | None = None,
    ) -> None:
        if (
            not isinstance(approved_mode, int)
            or isinstance(approved_mode, bool)
            or not 1 <= approved_mode <= 255
        ):
            raise ValueError("verified nonzero mode_machine is required")
        if len(gains) != 35 or len(modes) != 35 or any(mode not in (0, 1) for mode in modes):
            raise ValueError("verified complete 35-entry gain/mode profile is required")
        if any(
            len(pair) != 2 or any(not math.isfinite(v) or v < 0 for v in pair) for pair in gains
        ):
            raise ValueError("gains must be finite and nonnegative")
        if any(gains[i][0] <= 0 or gains[i][1] <= 0 or modes[i] != 1 for i in range(15, 29)):
            raise ValueError("both arms require verified position-control gains and modes")
        super().__init__(bundle_root, source=source, clock=clock)

        def gate() -> None:
            require_ownership()
            if self._fault is not None:
                raise SafetyFaultError("corrupt state")
            if self._positions is not None:
                if any(
                    not lower <= q <= upper
                    for q, (lower, upper) in zip(
                        self._positions[15:29], self.config["joint_limits_rad"]
                    )
                ):
                    raise SafetyFaultError("measured arm position outside verified limits")

        self.adapter = _ApprovedAdapter(
            clock=clock,
            approved_mode=approved_mode,
            gains=gains,
            modes=modes,
            require_ownership=gate,
        )
        profile_digest = digest_json(
            {
                "profile": self.config,
                "tools": self.tools,
                "control": {"mode": approved_mode, "gains": gains, "modes": modes},
            }
        )
        self.planner = G1DPlanner(
            kinematics=self.kinematics,
            clock=clock,
            skill_version=self.skill_version,
            runtime_instance_id=self.instance_id,
            profile_digest=profile_digest,
            max_joint_velocity_rad_per_s=self.config["max_joint_velocity_rad_per_s"],
            max_joint_acceleration_rad_per_s2=self.config["max_joint_acceleration_rad_per_s2"],
            max_joint_jerk_rad_per_s3=self.config["max_joint_jerk_rad_per_s3"],
            joint_limits_rad=self.config["joint_limits_rad"],
            base_frame=self.config["base_frame"],
            require_current_state=True,
        )
        self.dex1 = dex1
        self.gripper_status = getattr(sink, "gripper_status", None)
        self.executor = G1DExecutor(
            adapter=self.adapter,
            planner=self.planner,
            clock=clock,
            sink=sink,
            skill_version=self.skill_version,
            runtime_instance_id=self.instance_id,
            profile_digest=profile_digest,
            journal_path=journal_path,
            dex1=dex1,
        )
        self.control = G1DControlLoop(self.executor, self.poll)

    def poll(self) -> None:
        super().poll()
        if self._fault is not None:
            invocation = self.executor.active_invocation()
            if invocation is not None and not invocation.status.is_terminal:
                self.executor.mark_unknown(invocation.invocation_id, reason="state_corrupt")

    def state(self) -> dict[str, Any]:
        result = super().state()
        state = self.executor.query_state()
        if result["safety_gate"] in (
            "state_corrupt",
            "state_stale",
            "state_unavailable",
            "state_fault",
        ):
            state["action_ready"] = False
            state["safety_gate"] = result["safety_gate"]
        result = {**result, **state}
        if self.gripper_status is not None:
            result["gripper_control"] = self.gripper_status()
        return result

    def _validate_dex1(self, arguments: dict[str, Any]) -> None:
        requested = {
            side: arguments[side].get("dex1", {}).get("opening") for side in ("left", "right")
        }
        if any(value is not None for value in requested.values()):
            if self.dex1 is None:
                raise PlannerError("requested Dex1 service is unavailable")
            self.dex1.require_ready_for(requested)

    def plan_pose(self, arguments: dict[str, Any]) -> dict[str, Any]:
        active = self.executor.active_invocation()
        if active is not None and not active.status.is_terminal:
            raise PlannerError("planning is unavailable while the atomic operation is active")
        # The inherited planner needs the observed-state gate; control admission
        # remains independently enforced by the executor and supervisor.
        return self.executor.while_idle(lambda: G1DReadOnlyRuntime.plan_pose(self, arguments))

    def plan_gripper(self, arguments: dict[str, Any]) -> dict[str, Any]:
        def plan():
            with self.lock:
                self.adapter.require_ready()
                if self.dex1 is None:
                    raise PlannerError("requested Dex1 service is unavailable")
                self.dex1.require_ready_for(arguments)
                assert self._positions is not None
                return self._plan_response(self.planner.plan_gripper(
                    openings=arguments, current_q=self._positions))
        return self.executor.while_idle(plan)
