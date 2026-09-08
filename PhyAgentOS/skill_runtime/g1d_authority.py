"""Robot-host supervisor lease client; the Skill cannot approve its own handoff."""

from __future__ import annotations

import json
import socket
import threading
import time
import uuid
from pathlib import Path

from PhyAgentOS.skill_runtime.g1d_adapter import SafetyFaultError


class SupervisorAuthority:
    """Refresh an exclusive controller lease outside the 500 Hz writer.

    A local supervisor must implement the documented acquire/renew contract,
    enforcing exclusivity against *all* robot controllers. The client neither
    releases MotionSwitcher modes nor restores a previous controller.
    """

    def __init__(self, socket_path: Path, profile_digest: str):
        self.socket_path = socket_path
        self.profile_digest = profile_digest
        self.owner_id = f"g1d-controller-{uuid.uuid4().hex}"
        self._valid_until = 0.0
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="g1d-authority", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def require(self) -> None:
        with self._lock:
            if time.monotonic() >= self._valid_until:
                raise SafetyFaultError("approved exclusive controller lease unavailable")

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1)
        with self._lock:
            self._valid_until = 0.0

    def _run(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            expires = 0.0
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stream:
                    stream.settimeout(0.02)
                    stream.connect(str(self.socket_path))
                    stream.sendall(
                        (
                            json.dumps(
                                {
                                    "operation": "acquire_or_renew",
                                    "owner_id": self.owner_id,
                                    "profile_digest": self.profile_digest,
                                    "lease_ms": 50,
                                }
                            )
                            + "\n"
                        ).encode()
                    )
                    raw = bytearray()
                    while b"\n" not in raw and len(raw) < 4096:
                        data = stream.recv(4096 - len(raw))
                        if not data:
                            break
                        raw.extend(data)
                    reply = json.loads(raw)
                    if (
                        reply.get("owner_id") == self.owner_id
                        and reply.get("profile_digest") == self.profile_digest
                        and all(
                            reply.get(key) is True
                            for key in (
                                "exclusive",
                                "handoff_approved",
                                "operator_ready",
                                "constraints_verified",
                            )
                        )
                        and reply.get("lease_ms") == 50
                    ):
                        expires = started + 0.05
            except (OSError, ValueError, TypeError):
                pass
            with self._lock:
                self._valid_until = expires
            self._stop.wait(0.01)
