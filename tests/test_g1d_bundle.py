from __future__ import annotations

import json
from pathlib import Path

import pytest

from PhyAgentOS.skill_runtime.archive import ArchiveValidator, sha256_file
from PhyAgentOS.skill_runtime.manifest import load_manifest
from PhyAgentOS.skill_runtime.mock_runtime import MockRuntimeError, MockSkillRuntime
from scripts.package_skill import package

BUNDLE = Path(__file__).parents[1] / "bundles" / "g1d-manipulation"


def test_g1d_bundle_manifest_has_one_real_profile_and_four_tools() -> None:
    manifest = load_manifest(BUNDLE / "skill.yaml")

    assert manifest.manifest_version == 2
    assert manifest.name == "g1d-manipulation"
    assert set(manifest.profiles) == {"real-g1d"}
    assert manifest.profiles["real-g1d"].dataflow == Path(
        "profiles/real-g1d/dataflow.yaml"
    )
    assert manifest.required_tools == (
        "g1d.dual_arm.plan_pose",
        "g1d.dual_arm.execute_pose",
        "g1d.dual_arm.state",
        "g1d.dual_arm.stop",
    )


def test_tool_specs_are_exactly_four_and_strict() -> None:
    specs = json.loads((BUNDLE / "tools" / "tools.json").read_text())
    assert set(specs) == {
        "g1d.dual_arm.plan_pose",
        "g1d.dual_arm.execute_pose",
        "g1d.dual_arm.state",
        "g1d.dual_arm.stop",
    }
    assert {spec["semantics"] for spec in specs.values()} == {"query", "action"}
    assert all(spec["input_schema"]["additionalProperties"] is False for spec in specs.values())
    assert specs["g1d.dual_arm.plan_pose"]["robot_frame_profile"]["orientation"] == "normalized_xyzw"
    assert specs["g1d.dual_arm.plan_pose"]["robot_frame_profile"]["position_unit"] == "m"


def test_mock_runtime_lifecycle_and_gateway_context_are_robot_free() -> None:
    runtime = MockSkillRuntime(BUNDLE)

    assert runtime.status().status == "stopped"
    started = runtime.start("real-g1d")
    assert started.status == "running"
    inspection = runtime.inspect()
    assert inspection["profile"] == "real-g1d"
    assert set(inspection["tools"]) == {
        "g1d.dual_arm.plan_pose",
        "g1d.dual_arm.execute_pose",
        "g1d.dual_arm.state",
        "g1d.dual_arm.stop",
    }
    assert runtime.gateway_tools()["ok"] is True
    assert runtime.gateway_context("g1d.dual_arm.execute_pose")["data"]["ready"] is True
    assert runtime.status().runtime_ready is True
    assert runtime.status().action_ready is False
    assert runtime.stop().status == "stopped"
    assert runtime.stop().status == "stopped"


def test_mock_runtime_rejects_unknown_profile_and_hides_context_when_stopped() -> None:
    runtime = MockSkillRuntime(BUNDLE)

    with pytest.raises(MockRuntimeError, match="Unknown profile"):
        runtime.start("sim")
    assert runtime.gateway_tools()["ok"] is False
    assert runtime.gateway_context("g1d.dual_arm.state")["ok"] is False


def test_bundle_packages_with_verified_archive_inventory(tmp_path: Path) -> None:
    archive = package(BUNDLE, tmp_path)
    assert archive.name == "g1d-manipulation-0.3.0.tar.gz"
    extracted = tmp_path / "extracted"
    ArchiveValidator().extract(archive, extracted, expected_sha256=sha256_file(archive))
    assert (extracted / "skill.yaml").is_file()
    assert (extracted / "tools" / "tools.json").is_file()


def test_mock_runtime_requires_profile_assets_and_binaries(tmp_path: Path) -> None:
    broken = tmp_path / "g1d-manipulation"
    import shutil

    shutil.copytree(BUNDLE, broken)
    (broken / "profiles" / "real-g1d" / "kinematics.json").unlink()
    runtime = MockSkillRuntime(broken)
    with pytest.raises(MockRuntimeError):
        runtime.start("real-g1d")
