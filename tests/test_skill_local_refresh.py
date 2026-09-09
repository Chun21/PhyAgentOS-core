"""Local bundle updates must not be skipped because the manifest is unchanged."""

import os
import shutil
from pathlib import Path

import pytest

from PhyAgentOS.cli.commands import _install_skill_bundle
from PhyAgentOS.skill_runtime.archive import sha256_file
from PhyAgentOS.skill_runtime.installer import SkillInstaller
from PhyAgentOS.skill_runtime.manifest import load_manifest
from scripts.package_skill import package


@pytest.mark.parametrize("change", ["dataflow", "damaged_file", "unchanged"])
def test_install_compares_bundle_payload(tmp_path, monkeypatch, change):
    from PhyAgentOS.config import paths

    for key in os.environ:
        if key.lower().endswith("_proxy"):
            monkeypatch.delenv(key)
    monkeypatch.setattr(paths, "get_config_path", lambda: tmp_path / "instance/config.json")
    bundle = tmp_path / "source/g1d-manipulation"
    shutil.copytree(Path(__file__).parents[1] / "bundles/g1d-manipulation", bundle)
    archive = package(bundle, tmp_path / "archives")
    installed = SkillInstaller().install(archive)
    relative = Path("profiles/real-g1d/dataflow.yaml")
    if change == "dataflow":
        with (bundle / relative).open("a") as output:
            output.write("\n# updated launch environment\n")
        archive = package(bundle, tmp_path / "archives", force=True)
    elif change == "damaged_file":
        (installed.bundle_root / relative).write_text("nodes: []\n")
    assert installed == load_manifest(bundle / "skill.yaml")

    _install_skill_bundle(archive, expected_sha256=sha256_file(archive))

    assert (installed.bundle_root / relative).read_bytes() == (bundle / relative).read_bytes()
    backups = installed.bundle_root.parent / ".backups/g1d-manipulation"
    assert backups.exists() == (change != "unchanged")


def test_startup_reports_current_control_rejection_not_old_log(tmp_path, monkeypatch):
    from PhyAgentOS.skill_runtime.manager import RuntimeManager, RuntimeManagerError
    from PhyAgentOS.skill_runtime.state import RuntimeStateStore

    manager = RuntimeManager(
        runtime_root=tmp_path / "runtime", logs_root=tmp_path,
        state_store=RuntimeStateStore(tmp_path / "state"), health_timeout_s=.1,
    )
    flow = "test-flow"
    log = tmp_path / f"{flow}-dora.log"
    log.write_text("SafetyFaultError: stale failure from an older launch\n")
    offset = log.stat().st_size
    manifest = load_manifest(Path(__file__).parents[1] / "bundles/g1d-manipulation/skill.yaml")
    monkeypatch.setattr(manager, "_flow_running", lambda _: True)
    monkeypatch.setattr(manager, "_gateway_snapshot", lambda _: {"ok": True})
    monkeypatch.setattr(manager, "_tool_context_readiness", lambda _: dict.fromkeys(manifest.required_tools, True))
    manager._wait_until_ready(manifest, flow, log_offset=offset)
    with log.open("a") as output:
        output.write("SafetyFaultError: handoff incomplete: other writers=['peer']\n")
    with pytest.raises(RuntimeManagerError, match="handoff incomplete"):
        manager._wait_until_ready(manifest, flow, log_offset=offset)
