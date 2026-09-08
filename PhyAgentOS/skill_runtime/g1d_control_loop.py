"""Dedicated monotonic 500 Hz execution scheduler; no catch-up bursts."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

from PhyAgentOS.skill_runtime.g1d_executor import G1DExecutor


class G1DControlLoop:
    """Poll controlled DDS and advance the executor independently of Gateway IO.

    The injected poll boundary must be nonblocking. This is a best-effort
    userspace scheduler: measured native-host jitter is an acceptance gate,
    never a guarantee inferred from the requested period.
    """

    def __init__(self, executor: G1DExecutor, poll: Callable[[], None]) -> None:
        self.executor = executor
        self.poll = poll
        self.error: BaseException | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="g1d-500hz", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=5)
        if self._thread.is_alive():
            raise RuntimeError("control loop did not stop; physical outcome unresolved")
        self.executor.close()

    def _run(self) -> None:
        deadline = time.monotonic()
        try:
            while not self._stop.is_set():
                self.poll()
                self.executor.tick()
                deadline += 0.002
                now = time.monotonic()
                if deadline <= now:
                    # Preserve phase while skipping missed slots; never replay them.
                    deadline += (int((now - deadline) / 0.002) + 1) * 0.002
                self._stop.wait(max(0.0, deadline - time.monotonic()))
        except BaseException as error:
            self.error = error
            invocation = self.executor.active_invocation()
            if invocation is not None and not invocation.status.is_terminal:
                self.executor.mark_unknown(invocation.invocation_id, reason="control_loop_error")
