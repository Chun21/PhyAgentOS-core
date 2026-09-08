"""Create a verified G1_D Bundle containing the locked aarch64 dependency payload."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.build_g1d_node import BUNDLE, build
from scripts.package_skill import package


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gateway-archive", type=Path)
    parser.add_argument("--dependency-cache", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("dist/skills"))
    args = parser.parse_args()
    build(args.gateway_archive)
    subprocess.run(
        [
            sys.executable,
            str(BUNDLE / "dependencies/install.py"),
            "--download-only",
            "--cache",
            str(args.dependency_cache),
        ],
        check=True,
    )
    with tempfile.TemporaryDirectory(prefix="g1d-release-") as temporary:
        stage = Path(temporary) / "bundle"
        shutil.copytree(BUNDLE, stage)
        shutil.copytree(args.dependency_cache, stage / "dependencies/cache")
        print(package(stage, args.output_dir, force=True))


if __name__ == "__main__":
    main()
