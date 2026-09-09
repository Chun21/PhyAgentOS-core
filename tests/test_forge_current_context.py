"""Runtime upgrades must be visible even when the conversation contains old claims."""

import shutil
from pathlib import Path
from types import SimpleNamespace

from PhyAgentOS.agent.context import ContextBuilder
from PhyAgentOS.forge.binding import ForgeSkillBindingResolver
from PhyAgentOS.skill_runtime.catalog import SkillCatalog


def test_prompt_refreshes_runtime_version_and_skill_without_erasing_history(tmp_path):
    root = tmp_path / "skills"
    bundle = root / "g1d-manipulation"
    shutil.copytree(Path(__file__).parents[1] / "bundles/g1d-manipulation", bundle)
    catalog = SkillCatalog(root)
    manifest = catalog.get("g1d-manipulation")
    runtime = SimpleNamespace(skill_name=manifest.name, skill_version=manifest.version,
                              profile="real-g1d", runtime_instance_id="new-runtime")
    selected = [runtime]
    resolver = ForgeSkillBindingResolver(SimpleNamespace(current=lambda: selected[0]), catalog=catalog)
    builder = ContextBuilder(tmp_path / "workspace", forge_context_provider=resolver.current_context)
    (builder.workspace / "TOOLS.md").write_text("Historical observation: 0.3.1 only supports the right arm")
    history = [{"role": "user", "content": "wave left"},
               {"role": "assistant", "content": "0.3.1 cannot wave left"}]
    messages = builder.build_messages(history, "wave left again")
    assert messages[1:3] == history
    prompt = messages[0]["content"]
    assert f"version: {manifest.version}" in prompt
    assert 'gesture_arm: "left"' in prompt
    assert "not live action readiness" in prompt
    assert prompt.index("# Current Forge Runtime") > prompt.index("Historical observation")
    assert "Do not claim to have rechecked without a new tool result" in prompt
    selected[0] = None
    next_prompt = builder.build_messages(history, "now?")[0]["content"]
    assert "No managed Forge Runtime is currently selected" in next_prompt
    assert "runtime_instance_id: new-runtime" not in next_prompt


def test_changed_installed_version_does_not_advertise_new_capability(tmp_path):
    source = Path(__file__).parents[1] / "bundles"
    runtime = SimpleNamespace(skill_name="g1d-manipulation", skill_version="old",
                              profile="real-g1d", runtime_instance_id="old-runtime")
    resolver = ForgeSkillBindingResolver(SimpleNamespace(current=lambda: runtime), catalog=SkillCatalog(source))
    context = resolver.current_context()
    assert "Installed Skill version differs" in context
    assert "gesture_arm" not in context
