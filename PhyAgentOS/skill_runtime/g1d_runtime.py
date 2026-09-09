"""Read-only G1_D Runtime: state subscription and real-model bilateral planning."""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
import uuid
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import yaml

from PhyAgentOS.skill_runtime.g1d_adapter import (
    AdapterError,
    G1DAdapter,
    LowStateSource,
    StaleStateError,
)
from PhyAgentOS.skill_runtime.g1d_bridge import BridgeError
from PhyAgentOS.skill_runtime.g1d_kinematics_pin import default_urdf_path
from PhyAgentOS.skill_runtime.g1d_kinematics_unirobot import UniRobotKinematics, model_directory
from PhyAgentOS.skill_runtime.g1d_planner import (
    G1DPlanner,
    PlannerError,
    digest_json,
    load_kinematics_profile,
)


class G1DReadOnlyRuntime:
    """Own one immutable model/profile and serialize its Pinocchio workspace."""

    def __init__(
        self,
        bundle_root: Path,
        *,
        source: LowStateSource,
        profile: str = "real-g1d",
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if profile != "real-g1d":
            raise ValueError(f"unknown profile: {profile}")
        self.root = bundle_root
        manifest = yaml.safe_load((bundle_root / "skill.yaml").read_text())
        if not isinstance(manifest, dict) or not isinstance(manifest.get("version"), str):
            raise ValueError("Skill manifest requires a version")
        self.skill_version = manifest["version"]
        self.clock = clock
        self.source = source
        self.instance_id = f"g1d-{uuid.uuid4().hex}"
        self.lock = threading.RLock()
        self.adapter = G1DAdapter(clock=clock)
        self._fault: str | None = None
        self._received_at: float | None = None
        self._tick: int | None = None
        self._positions: tuple[float, ...] | None = None
        self.config = load_kinematics_profile(bundle_root / "profiles/real-g1d/kinematics.json")
        fixed = {
            "profile": "g1d-real-v1",
            "platform": "unitree-g1d",
            "arm_dof": 7,
            "plan_ttl_s": 5.0,
            "position_tolerance_m": 0.02,
            "orientation_tolerance_deg": 5.0,
            "max_joint_velocity_rad_per_s": 0.5,
            "max_joint_acceleration_rad_per_s2": 2.0,
            "max_joint_jerk_rad_per_s3": 10.0,
        }
        if any(self.config.get(key) != value for key, value in fixed.items()):
            raise ValueError("unsupported or invalid planning profile")
        if self.config.get("trajectory") != {
            "interpolation": "quintic",
            "minimum_duration_s": 1.0,
            "sample_hz": 500,
            "writer_jitter_p99_ms": 0.5,
            "watchdog_missed_cycles": 10,
        }:
            raise ValueError("unsupported trajectory profile")
        if self.config.get("ik_solver") != "unirobot.g1d_dex1_casadi_ipopt_v1":
            raise ValueError("unsupported profile solver")
        if (
            self.config.get("base_frame") != "unirobot_g1d_fixed_base"
            or self.config.get("model_frame") != "unirobot_g1d_fixed_base"
        ):
            raise ValueError("unsupported profile frame")
        urdf = default_urdf_path()
        if hashlib.sha256(urdf.read_bytes()).hexdigest() != self.config.get("urdf_sha256"):
            raise ValueError("profile URDF hash mismatch")
        model = model_directory() / "model.json"
        if hashlib.sha256(model.read_bytes()).hexdigest() != self.config.get("model_sha256"):
            raise ValueError("reference model hash mismatch")
        self.kinematics = UniRobotKinematics()
        limits = self.config.get("joint_limits_rad")
        expected = list(
            zip(self.kinematics.model.lowerPositionLimit, self.kinematics.model.upperPositionLimit)
        )
        if (
            not isinstance(limits, list)
            or len(limits) != 14
            or any(
                not isinstance(pair, list)
                or len(pair) != 2
                or any(
                    not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v)
                    for v in pair
                )
                or any(abs(a - b) > 1e-10 for a, b in zip(pair, bounds))
                for pair, bounds in zip(limits, expected)
            )
        ):
            raise ValueError("profile joint limits do not match verified URDF")
        self.tools = json.loads((bundle_root / "tools/tools.json").read_text())
        self.planner = G1DPlanner(
            kinematics=self.kinematics,
            clock=clock,
            skill_version=self.skill_version,
            runtime_instance_id=self.instance_id,
            profile_digest=digest_json({"profile": self.config, "tools": self.tools}),
            joint_limits_rad=limits,
            base_frame=self.config["base_frame"],
            require_current_state=True,
            max_joint_velocity_rad_per_s=self.config["max_joint_velocity_rad_per_s"],
            max_joint_acceleration_rad_per_s2=self.config["max_joint_acceleration_rad_per_s2"],
            max_joint_jerk_rad_per_s3=self.config["max_joint_jerk_rad_per_s3"],
        )

    def poll(self) -> None:
        with self.lock:
            try:
                item = self.source.read()
                if item is None:
                    return
                frame, received_at = item
                if any(not math.isfinite(q) for q in frame.positions):
                    raise ValueError("state joint positions must be finite")
                if self._tick is not None and not 0 < (frame.tick - self._tick) % (2**32) < 2**31:
                    return
                if not math.isfinite(received_at) or received_at > self.clock():
                    raise ValueError("invalid state timestamp")
                self.adapter.ingest(frame, received_at=received_at)
                self._positions = frame.positions
                self._received_at = received_at
                self._tick = frame.tick
                self._fault = None
            except (AdapterError, BridgeError, ValueError, TypeError) as error:
                self._fault = str(error)

    def state(self) -> dict[str, Any]:
        with self.lock:
            result: dict[str, Any] = {
                "runtime_instance_id": self.instance_id,
                "skill_version": self.skill_version,
                "runtime_ready": True,
                "action_ready": False,
                "safety_gate": "state_unavailable",
                "dex1": {side: {"status": "absent"} for side in ("left", "right")},
            }
            if self._received_at is not None:
                result["state_age_ms"] = max(0.0, (self.clock() - self._received_at) * 1000)
            if self._fault is not None:
                result["safety_gate"] = "state_corrupt"
                return result
            try:
                state = self.adapter.query_state()
            except StaleStateError:
                result["safety_gate"] = "state_stale"
                return result
            except AdapterError:
                return result
            result["mode_machine"] = state.mode_machine
            # HG state cannot attest E-stop, ownership, or physical clearance.
            result["safety_gate"] = (
                "state_fault" if state.safety_gate == "fault" else "read_only_unverified"
            )
            for side, arm in (("left", state.left_arm), ("right", state.right_arm)):
                result[f"{side}_arm"] = {"joint_positions_rad": [motor.q for motor in arm]}
            poses = self.kinematics.solve_fk(
                result["left_arm"]["joint_positions_rad"],
                result["right_arm"]["joint_positions_rad"],
            )
            result["end_effector_poses"] = {
                side: {"frame_id": self.config["base_frame"],
                       "position_m": list(pose.position_m),
                       "orientation_xyzw": list(pose.orientation_xyzw)}
                for side, pose in zip(("left", "right"), poses)
            }
            return result

    def _validate_dex1(self, arguments: dict[str, Any]) -> None:
        if any("dex1" in arguments[side] for side in ("left", "right")):
            raise PlannerError("requested Dex1 readiness is unavailable in the read-only profile")

    def plan_pose(self, arguments: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            self._validate_dex1(arguments)
            state = G1DReadOnlyRuntime.state(self)
            if state["safety_gate"] != "read_only_unverified":
                raise PlannerError(f"valid planning state required: {state['safety_gate']}")
            assert self._positions is not None
            self.kinematics.set_reference_q(self._positions[15:29])
            if "gesture_arm" in arguments and arguments.get("gesture") != "wave":
                raise PlannerError("gesture_arm requires gesture=wave")
            targets = dict(left=arguments["left"], right=arguments["right"], current_q=self._positions)
            if arguments.get("gesture") == "wave":
                plan = self.planner.plan_wave(**targets, arm=arguments.get("gesture_arm", "right"))
            else:
                plan = self.planner.plan_pose(**targets)
            if not plan.checks_passed:
                raise PlannerError("bilateral plan validation failed")
            if "deadline_s" in arguments and plan.trajectory.duration_s > arguments["deadline_s"]:
                raise PlannerError("trajectory exceeds requested deadline")
            binding = asdict(plan.binding)
            binding.pop(
                "profile_digest"
            )  # Kept internally; the public schema is intentionally narrow.
            return {
                "plan_id": plan.plan_id,
                "expires_at": datetime.fromtimestamp(
                    time.time() + max(0.0, plan.expires_at - self.clock()), timezone.utc
                ).isoformat(),
                "binding": binding,
            }
