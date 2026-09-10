"""Build a single relocatable zipapp for the existing NodeInstaller contract."""

from __future__ import annotations

import gzip
import hashlib
import io
import json
import tarfile
import urllib.request
import zipfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / "bundles/g1d-manipulation"
GATEWAY_COMMIT = "49caba94d641aff4664fbdf9d4c8b1402add1ce6"
GATEWAY_SHA256 = "74b0a9fce14abb2b44ee3c85d669c7a60e94a8703c6f9cd93c186e9d22780246"
GATEWAY_URL = (
    f"https://codeload.github.com/Forgelab-Robotics/adapter-forge-gateway/tar.gz/{GATEWAY_COMMIT}"
)

BOOTSTRAP = """import sys
import tempfile
import zipfile
from pathlib import Path

# The NodeInstaller verifies the complete executable hash before launch.
# Extract only this executable's immutable payload, for native URDF file access.
with tempfile.TemporaryDirectory(prefix="g1d-runtime-") as directory:
    with zipfile.ZipFile(sys.argv[0]) as archive:
        archive.extractall(directory)
    sys.path.insert(0, directory)
    from PhyAgentOS.skill_runtime.g1d_gateway import main
    main()
"""


def build(gateway_archive: Path | None = None) -> Path:
    raw = (
        gateway_archive.read_bytes()
        if gateway_archive
        else urllib.request.urlopen(GATEWAY_URL, timeout=60).read()
    )
    if hashlib.sha256(raw).hexdigest() != GATEWAY_SHA256:
        raise ValueError("Gateway source hash mismatch")
    files = {
        "__main__.py": BOOTSTRAP.encode(),
        "PhyAgentOS/__init__.py": b"",
        "PhyAgentOS/skill_runtime/__init__.py": b"",
    }
    for name in (
        "g1d_adapter",
        "g1d_planner",
        "g1d_trajectory",
        "g1d_executor",
        "g1d_dex1",
        "g1d_dex1_bridge",
        "g1d_gripper_hold",
        "g1d_camera",
        "g1d_control_loop",
        "g1d_control_session",
        "agent_lease",
        "g1d_motion_switcher",
        "g1d_action_endpoint",
        "g1d_authority",
        "g1d_execution_runtime",
        "g1d_bridge",
        "g1d_kinematics_pin",
        "g1d_kinematics_unirobot",
        "g1d_runtime",
        "g1d_gateway",
    ):
        relative = f"PhyAgentOS/skill_runtime/{name}.py"
        files[relative] = (ROOT / relative).read_bytes()
    for name in ("g1_d.urdf", "README.md", "LICENSE"):
        relative = f"assets/robots/g1d/{name}"
        files[relative] = (ROOT / relative).read_bytes()
    for path in sorted((ROOT / "assets/robots/g1d/unirobot_ik").iterdir()):
        if path.is_file():
            files[str(path.relative_to(ROOT))] = path.read_bytes()
    for path in [BUNDLE / "tools/tools.json", BUNDLE / "profiles/real-g1d/kinematics.json",
                 BUNDLE / "profiles/real-g1d/camera.json"]:
        files[f"bundle/{path.relative_to(BUNDLE)}"] = path.read_bytes()
    # Keep the standalone fallback version aligned without embedding a
    # self-referential archive digest from the complete Skill lock.
    skill_metadata = yaml.safe_load((BUNDLE / "skill.yaml").read_text())
    files["bundle/skill.yaml"] = yaml.safe_dump({
        "name": skill_metadata["name"], "version": skill_metadata["version"]
    }).encode()
    with tarfile.open(fileobj=io.BytesIO(raw), mode="r:gz") as archive:
        for member in archive:
            parts = Path(member.name).parts[1:]
            if not member.isfile():
                continue
            # Preserve upstream sources/notices except the documented deadline patch.
            if (
                len(parts) >= 3
                and parts[0] == "src"
                and parts[1] in ("forge_tool", "forge_gateway")
                and parts[-1].endswith(".py")
            ):
                target = "/".join(parts[1:])
            elif len(parts) == 1 and parts[0] in ("LICENSE", "NOTICE", "THIRD_PARTY_NOTICES.md"):
                target = f"licenses/forge-gateway/{parts[0]}"
            else:
                continue
            source = archive.extractfile(member)
            assert source is not None
            data = source.read()
            if target == "forge_gateway/services/tool_gateway_service.py":
                # Narrow, version-pinned downstream fix: handshake timeout is not
                # the physical operation deadline. See patches/README.md.
                text = data.decode()
                anchor = "        deadline_ms = (\n            None\n"
                if text.count(anchor) != 1:
                    raise ValueError("Gateway deadline patch anchor mismatch")
                text = text.replace(
                    anchor,
                    """        operation_timeout = effective_timeout
        if spec.tool_id == "g1d.dual_arm.execute_pose":
            seconds = arguments.get("operation_deadline_s", 30.0)
            if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not 1 <= seconds <= 120:
                raise ValueError("operation_deadline_s must be within [1, 120]")
            operation_timeout = int(seconds * 1000)
"""
                    + anchor,
                )
                expression = "int(time.time() * 1_000) + effective_timeout,"
                offset = text.index(anchor)
                if expression not in text[offset:]:
                    raise ValueError("Gateway deadline patch expression mismatch")
                data = (
                    text[:offset]
                    + text[offset:].replace(
                        expression, "int(time.time() * 1_000) + operation_timeout,", 1
                    )
                ).encode()
            files[target] = data
    files["licenses/forge-gateway/source.json"] = json.dumps(
        {"commit": GATEWAY_COMMIT, "sha256": GATEWAY_SHA256, "url": GATEWAY_URL}
    ).encode()
    files["licenses/PhyAgentOS-LICENSE"] = (ROOT / "LICENSE").read_bytes()
    buffer = io.BytesIO(b"#!/usr/bin/env python3\n")
    buffer.seek(0, 2)
    with zipfile.ZipFile(buffer, "a", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in sorted(files.items()):
            info = zipfile.ZipInfo(name, (2026, 1, 1, 0, 0, 0))
            archive.writestr(info, data, compress_type=zipfile.ZIP_DEFLATED)
    executable = buffer.getvalue()
    (BUNDLE / "artifacts/g1d-runtime").write_bytes(executable)
    (BUNDLE / "artifacts/g1d-runtime").chmod(0o755)
    manifest_path = BUNDLE / "skill.yaml"
    manifest = yaml.safe_load(manifest_path.read_text())
    node_version = manifest["artifacts"]["nodes"]["g1d-runtime"]["version"]
    archive_path = BUNDLE / f"artifacts/g1d-runtime-{node_version}.tar.gz"
    with archive_path.open("wb") as output:
        with gzip.GzipFile(fileobj=output, mode="wb", filename="", mtime=0) as zipped:
            with tarfile.open(fileobj=zipped, mode="w") as archive:
                member = tarfile.TarInfo("g1d-runtime")
                member.size, member.mode, member.mtime = len(executable), 0o755, 0
                archive.addfile(member, io.BytesIO(executable))
    manifest["artifacts"]["nodes"]["g1d-runtime"]["sha256"] = hashlib.sha256(
        archive_path.read_bytes()
    ).hexdigest()
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False))
    return archive_path


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gateway-archive", type=Path)
    args = parser.parse_args()
    print(build(args.gateway_archive))
