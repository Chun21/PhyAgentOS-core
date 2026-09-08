"""Bounded read-only G1_D DDS probe; never constructs command publishers."""
from __future__ import annotations

import argparse
import json
import time

from PhyAgentOS.skill_runtime.g1d_bridge import CycloneLowStateSource


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--seconds', type=float, default=5)
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error('seconds must be within (0, 60]')
    source = CycloneLowStateSource(domain_id=args.domain)
    deadline = time.monotonic() + args.seconds
    count = 0
    last = None
    while time.monotonic() < deadline:
        sample = source.read()
        if sample is not None:
            count += 1
            last = sample
        time.sleep(.002)
    result = {'samples': count, 'domain': args.domain, 'read_only': True}
    if last is not None:
        frame, received = last
        result.update(mode_machine=frame.mode_machine, tick=frame.tick,
                      state_age_ms=(time.monotonic()-received)*1000,
                      safety_ok=frame.safety_ok, positions_rad=frame.positions)
    print(json.dumps(result))
    return 0 if last is not None else 2


if __name__ == '__main__':
    raise SystemExit(main())
