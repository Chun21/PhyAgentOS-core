"""Robot-free Runtime seam used to exercise Bundle and Gateway lifecycle contracts.

This module intentionally never imports a robot SDK, DDS client, or subprocess launcher.
It is suitable for archive/runtime tests and local contract inspection only.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from PhyAgentOS.skill_runtime.manifest import ManifestError, SkillManifest, load_manifest


class MockRuntimeError(RuntimeError):
    """Raised when the mock Runtime lifecycle contract is violated."""


@dataclass(frozen=True)
class MockRuntimeStatus:
    status: str
    profile: str | None = None
    runtime_ready: bool = False
    action_ready: bool = False
    last_error: str | None = None


class MockSkillRuntime:
    """Deterministic Bundle/Gateway lifecycle without physical robot access."""

    def __init__(self, bundle_root: Path) -> None:
        self.bundle_root = bundle_root.expanduser().resolve()
        try:
            self.manifest: SkillManifest = load_manifest(self.bundle_root / "skill.yaml")
        except ManifestError as exc:
            raise MockRuntimeError(str(exc)) from exc
        tools_path = self.bundle_root / "tools" / "tools.json"
        try:
            tools = json.loads(tools_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise MockRuntimeError("Bundle tools/tools.json is missing or invalid") from exc
        if not isinstance(tools, dict):
            raise MockRuntimeError("Bundle tools/tools.json must be an object")
        if set(tools) != set(self.manifest.required_tools):
            raise MockRuntimeError("manifest required_tools and ToolSpecs do not match")
        expected_semantics = {
            "g1d.dual_arm.plan_pose": "query",
            "g1d.dual_arm.execute_pose": "action",
            "g1d.dual_arm.state": "query",
            "g1d.dual_arm.stop": "action",
            "g1d.dual_arm.plan_gripper": "query",
            "g1d.dual_arm.camera_state": "query",
            "g1d.dual_arm.camera_observe": "query",
        }
        if set(tools) != set(expected_semantics):
            raise MockRuntimeError("g1d Bundle must expose exactly seven ToolSpecs")
        for tool_id, semantics in expected_semantics.items():
            spec = tools[tool_id]
            if not isinstance(spec, dict) or spec.get("semantics") != semantics:
                raise MockRuntimeError(f"ToolSpec {tool_id!r} has invalid semantics")
            schema = spec.get("input_schema")
            if not isinstance(schema, dict) or schema.get("additionalProperties") is not False:
                raise MockRuntimeError(f"ToolSpec {tool_id!r} must use a strict input schema")
        self._tools: dict[str, dict[str, Any]] = tools
        self._status = MockRuntimeStatus("stopped")

    def start(self, profile: str) -> MockRuntimeStatus:
        if profile not in self.manifest.profiles:
            raise MockRuntimeError(f"Unknown profile {profile!r}")
        if self._status.status == "running":
            if self._status.profile != profile:
                raise MockRuntimeError("a different mock profile is already running")
            return self._status
        dataflow = self.manifest.resolve_bundle_path(self.manifest.profiles[profile].dataflow)
        if not dataflow.is_file():
            self._status = MockRuntimeStatus("failed", profile, last_error="dataflow is missing")
            raise MockRuntimeError("dataflow is missing from Bundle")
        runtime_profile = self.manifest.profiles[profile]
        for relative in runtime_profile.required_assets:
            if not self.manifest.resolve_bundle_path(relative).is_file():
                self._status = MockRuntimeStatus("failed", profile, last_error="asset is missing")
                raise MockRuntimeError(f"required asset is missing: {relative.as_posix()}")
        for relative in runtime_profile.required_binaries:
            candidate = self.manifest.resolve_bundle_path(relative)
            if not candidate.is_file():
                candidate = self.manifest.resolve_bundle_path(Path("artifacts") / relative)
            if not candidate.is_file():
                self._status = MockRuntimeStatus("failed", profile, last_error="binary is missing")
                raise MockRuntimeError(f"required binary is missing: {relative.as_posix()}")
            if candidate.stat().st_mode & 0o111 == 0:
                raise MockRuntimeError(f"required binary is not executable: {relative.as_posix()}")
        for lock in self.manifest.artifacts.nodes.values():
            archive = self.manifest.resolve_bundle_path(
                Path("artifacts") / f"{lock.artifact_id}.tar.gz"
            )
            if archive.is_file() and hashlib.sha256(archive.read_bytes()).hexdigest() != lock.sha256:
                raise MockRuntimeError(f"locked artifact digest mismatch: {lock.node_id}")
        self._status = MockRuntimeStatus("running", profile, runtime_ready=True, action_ready=False)
        return self._status

    def inspect(self) -> dict[str, Any]:
        return {
            "skill": self.manifest.name,
            "version": self.manifest.version,
            "profile": self._status.profile,
            "status": self._status.status,
            "gateway_url": self.manifest.gateway_url,
            "tools": sorted(self._tools),
            "runtime_ready": self._status.runtime_ready,
            "action_ready": self._status.action_ready,
        }

    def status(self) -> MockRuntimeStatus:
        return self._status

    def stop(self) -> MockRuntimeStatus:
        self._status = MockRuntimeStatus("stopped")
        return self._status

    def gateway_tools(self) -> dict[str, Any]:
        if self._status.status != "running":
            return {"ok": False, "error": {"code": "runtime_not_running"}}
        return {"ok": True, "data": {"tools": list(self._tools.values())}}

    def gateway_context(self, tool_id: str) -> dict[str, Any]:
        if tool_id not in self._tools:
            return {"ok": False, "error": {"code": "tool_not_found"}}
        if self._status.status != "running":
            return {"ok": False, "error": {"code": "runtime_not_running"}}
        return {
            "ok": True,
            "data": {
                "tool_id": tool_id,
                "ready": self._status.runtime_ready,
                "runtime_ready": True,
                "action_ready": self._status.action_ready,
                "binding_error": None,
            },
        }


__all__ = ["MockRuntimeError", "MockRuntimeStatus", "MockSkillRuntime"]
