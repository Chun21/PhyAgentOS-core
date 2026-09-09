"""Local CLI liveness lease shared with its supervised robot Runtime.

The heartbeat runs independently of model requests and terminal input. A unique
file per invocation prevents a later CLI from reviving an abandoned controller.
Both processes use the same host's monotonic clock.
"""

from __future__ import annotations

import math
import os
import threading
import time
from pathlib import Path
from uuid import uuid4

LEASE_TIMEOUT_S = 10.0
HEARTBEAT_INTERVAL_S = 1.0


def lease_deadline(path: Path, now: float) -> float:
    try:
        stamp = float(path.read_text(encoding="ascii"))
    except (OSError, ValueError) as exc:
        raise ValueError("PAOS agent disconnected: control lease unavailable") from exc
    if not math.isfinite(stamp) or not 0 <= now - stamp < LEASE_TIMEOUT_S:
        raise ValueError("PAOS agent heartbeat expired")
    return stamp + LEASE_TIMEOUT_S


class AgentControlLease:
    """Renew while the CLI is alive; revoke on any normal or exceptional exit."""

    def __init__(self, root: Path):
        self.path = root / f"agent-{uuid4().hex}.lease"
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error: str | None = None
        self.runtime = None

    def _write(self):
        temporary = self.path.with_suffix(".tmp")
        try:
            temporary.write_text(str(time.monotonic()), encoding="ascii")
            temporary.chmod(0o600)
            os.replace(temporary, self.path)
        finally:
            temporary.unlink(missing_ok=True)

    def _renew(self):
        try:
            while not self._stop.wait(HEARTBEAT_INTERVAL_S):
                self._write()
        except OSError as exc:
            self.error = str(exc)
            # Fail closed: a broken heartbeat cannot extend control authority.
            self.path.unlink(missing_ok=True)

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._write()
        self._thread = threading.Thread(target=self._renew, name="paos-control-lease", daemon=True)
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
        self.path.unlink(missing_ok=True)

    def __exit__(self, *_):
        self.close()
