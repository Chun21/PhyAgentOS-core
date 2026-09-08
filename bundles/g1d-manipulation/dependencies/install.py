"""Verify locked artifacts and install in an explicitly activated conda environment.

Run --download-only on a connected build host to populate the release Bundle.
Run without it on the aarch64 robot host, after creating the locked conda env.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def verify(path: Path, digest: str) -> bool:
    return path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest


def artifacts() -> list[dict]:
    return json.loads((ROOT / "wheels-linux-aarch64.json").read_text()) + json.loads(
        (ROOT / "sources.lock.json").read_text()
    )


def download(cache: Path, *, offline: bool) -> None:
    cache.mkdir(parents=True, exist_ok=True)
    for item in artifacts():
        path = cache / item["filename"]
        if verify(path, item["sha256"]):
            continue
        if offline:
            raise ValueError(f"missing or corrupt dependency: {path.name}")
        with urllib.request.urlopen(item["url"], timeout=60) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise ValueError(f"dependency hash mismatch: {path.name}")
        path.write_bytes(data)


def run(*command: str, env: dict[str, str] | None = None) -> None:
    subprocess.run(command, check=True, env=env)


def install(cache: Path) -> None:
    if platform.machine() not in ("aarch64", "arm64") or platform.system() != "Linux":
        raise RuntimeError("installation requires Linux aarch64")
    if sys.version_info[:2] != (3, 12) or not os.environ.get("CONDA_PREFIX"):
        raise RuntimeError("activate the locked CPython 3.12 conda environment first")
    if Path(sys.prefix).resolve() != Path(os.environ["CONDA_PREFIX"]).resolve():
        raise RuntimeError("python does not belong to the active conda environment")
    run(
        sys.executable,
        "-m",
        "pip",
        "install",
        "--no-index",
        "--find-links",
        str(cache),
        "--require-hashes",
        "-r",
        str(ROOT / "requirements-linux-aarch64.lock"),
    )
    prefix = Path(sys.prefix) / "g1d-dds"
    with tempfile.TemporaryDirectory(prefix="g1d-dds-build-") as temporary:
        directory = Path(temporary)
        with tarfile.open(cache / "cyclonedds-core-11.0.1.tar.gz") as archive:
            archive.extractall(directory, filter="data")
        build = directory / "build"
        run(
            "cmake",
            "-S",
            str(directory / "cyclonedds-11.0.1"),
            "-B",
            str(build),
            f"-DCMAKE_INSTALL_PREFIX={prefix}",
            "-DBUILD_DDSPERF=OFF",
            "-DBUILD_TESTING=OFF",
            "-DENABLE_SSL=OFF",
            "-DENABLE_SECURITY=OFF",
            "-DCMAKE_BUILD_TYPE=Release",
        )
        run("cmake", "--build", str(build), "--parallel", "2")
        run("cmake", "--install", str(build))
        env = {**os.environ, "CYCLONEDDS_HOME": str(prefix)}
        run(
            sys.executable,
            "-m",
            "pip",
            "install",
            "--no-index",
            "--no-deps",
            "--no-build-isolation",
            str(cache / "cyclonedds-11.0.1.tar.gz"),
            env=env,
        )
    run(sys.executable, "-m", "pip", "check")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--download-only", action="store_true")
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--cache", type=Path, default=ROOT / "cache")
    args = parser.parse_args()
    download(args.cache, offline=args.offline)
    if not args.download_only:
        install(args.cache)
