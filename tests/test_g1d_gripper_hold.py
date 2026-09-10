import pytest

from PhyAgentOS.skill_runtime.g1d_dex1 import Dex1CommandSample, Dex1Error, Dex1Integration
from PhyAgentOS.skill_runtime.g1d_gripper_hold import GripperHold


def test_rate_limit_continues_between_actions_freezes_and_drops_stale_side():
    now = [10.]
    dex = Dex1Integration(clock=lambda: now[0])

    class Bridge:
        samples = []
        competing = False

        def require_exclusive(self, side):
            if self.competing:
                raise Dex1Error("competing writer")

        def write(self, sample):
            self.samples.append(sample)

    bridge = Bridge()
    hold = GripperHold(bridge, dex, lambda: now[0])
    dex.publish("left", opening=.3, received_at=now[0])
    hold.write(Dex1CommandSample(now[0], dex.command("left", 1.)))
    for _ in range(100):
        now[0] += .021
        dex.publish("left", opening=hold.commanded["left"], received_at=now[0])
        hold.tick()
    assert bridge.samples[-1].command.opening == 1.
    assert all(s.command.side == "left" for s in bridge.samples)
    assert all(0 <= b.command.opening - a.command.opening <= .5 * (b.t_s - a.t_s) + 1e-9
               for a, b in zip(bridge.samples, bridge.samples[1:]))
    hold.write(Dex1CommandSample(now[0], dex.command("left", 0.)))
    hold.freeze()
    now[0] += .021
    hold.tick()
    assert bridge.samples[-1].command.opening == 1.
    count = len(bridge.samples)
    now[0] += .2
    hold.tick()
    assert not hold.targets and len(bridge.samples) == count
    dex.publish("left", opening=.5, received_at=now[0])
    now[0] += .021
    hold.tick()
    assert len(bridge.samples) == count  # No silent restart on recovered feedback.
    with pytest.raises(Dex1Error, match="newly admitted"):
        hold.write(Dex1CommandSample(now[0], dex.command("left", 0.)))
    hold.validate_plan({"left": 0.})
    bridge.competing = True
    with pytest.raises(Dex1Error, match="competing"):
        hold.write(Dex1CommandSample(now[0], dex.command("left", 0.)))
