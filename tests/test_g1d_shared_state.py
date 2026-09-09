import queue
import time

import pytest

from PhyAgentOS.skill_runtime.g1d_adapter import make_lowstate_frame
from PhyAgentOS.skill_runtime.g1d_bridge import BridgeCRCError, SharedLowStateSource


def test_shared_reader_preserves_freshness_and_propagates_corruption():
    items = queue.Queue()

    class Source:
        def read(self):
            try:
                item = items.get_nowait()
            except queue.Empty:
                return None
            if isinstance(item, Exception):
                raise item
            return item

    frame = make_lowstate_frame(mode_machine=5, tick=1, positions=[.1]*35)
    received = time.monotonic()
    items.put((frame, received))
    source = SharedLowStateSource(Source())
    source.start()
    try:
        deadline = time.monotonic() + 1
        while source.read() is None:
            assert time.monotonic() < deadline
            time.sleep(.002)
        for _ in range(100):
            assert source.read() == (frame, received)
        time.sleep(.11)
        assert source.read()[1] == received  # Readers cannot freshen an old sample.
        items.put(BridgeCRCError("bad CRC"))
        deadline = time.monotonic() + 1
        while source._error is None:
            assert time.monotonic() < deadline
            time.sleep(.002)
        with pytest.raises(BridgeCRCError):
            source.read()
    finally:
        source.close()
