"""Rate-limited Dex1 targets refreshed only while the arm session owns control."""

from PhyAgentOS.skill_runtime.g1d_dex1 import Dex1CommandSample, Dex1Error


class GripperHold:
    def __init__(self, bridge, integration, clock):
        self.bridge, self.integration, self.clock = bridge, integration, clock
        self.targets = {}
        self.commanded = {}
        self.errors = {}
        self._last = None

    def write(self, sample):
        side, opening = sample.command.side, sample.command.opening
        if side in self.errors:
            raise Dex1Error(f"Dex1 {side} requires a newly admitted plan: {self.errors[side]}")
        self.integration.require_ready_for({side: opening})
        self.bridge.require_exclusive(side)
        if side not in self.commanded:
            self.commanded[side] = self.integration.readiness(side).opening
        self.targets[side] = self.integration.command(side, opening).opening

    def validate_plan(self, openings):
        self.integration.require_ready_for(openings)
        for side, opening in openings.items():
            if opening is not None:
                self.bridge.require_exclusive(side)
        # Only explicit admission can clear a latched refresh fault. The old
        # Action's 500 Hz target stream must never silently re-arm a failed side.
        for side, opening in openings.items():
            if opening is not None:
                self.errors.pop(side, None)

    def state(self):
        return {side: {"command_active": side in self.targets,
                       "target_opening": self.targets.get(side),
                       "commanded_opening": self.commanded.get(side),
                       "error": self.errors.get(side)} for side in ("left", "right")}

    def freeze(self):
        self.targets = dict(self.commanded)

    def tick(self):
        now = self.clock()
        if self._last is not None and now - self._last < .02:
            return
        dt = min(.04, now - self._last) if self._last is not None else .02
        self._last = now
        for side, target in list(self.targets.items()):
            try:
                self.integration.require_ready_for({side: target})
                self.bridge.require_exclusive(side)
            except Dex1Error as exc:
                # No automatic resumption after state/ownership loss. The external
                # service enters BRAKE after its own 1 s command timeout.
                self.targets.pop(side, None)
                self.commanded.pop(side, None)
                self.errors[side] = str(exc)
                continue
            previous = self.commanded[side]
            opening = previous + max(-.5 * dt, min(.5 * dt, target - previous))
            self.bridge.write(Dex1CommandSample(now, self.integration.command(side, opening)))
            self.commanded[side] = opening
