from pathlib import Path

import pytest

from PhyAgentOS.skill_runtime.g1d_adapter import FakeLowStateSource, make_lowstate_frame
from PhyAgentOS.skill_runtime.g1d_runtime import G1DReadOnlyRuntime

BUNDLE = Path(__file__).parents[1] / "bundles/g1d-manipulation"


def test_readonly_state_requires_fresh_inputs_and_never_admits_motion():
    now = [100.0]
    source = FakeLowStateSource()
    runtime = G1DReadOnlyRuntime(BUNDLE, source=source, clock=lambda: now[0])
    assert runtime.state()["safety_gate"] == "state_unavailable"
    source.publish(
        make_lowstate_frame(mode_machine=7, tick=1, positions=[0.0] * 35), received_at=now[0]
    )
    runtime.poll()
    state = runtime.state()
    assert state["state_age_ms"] == 0
    assert state["mode_machine"] == 7
    assert len(state["left_arm"]["joint_positions_rad"]) == 7
    assert state["action_ready"] is False
    now[0] += 0.101
    assert runtime.state()["safety_gate"] == "state_stale"


def test_runtime_rejects_unknown_profile():
    with pytest.raises(ValueError, match="profile"):
        G1DReadOnlyRuntime(BUNDLE, source=FakeLowStateSource(), profile="unknown")


@pytest.mark.parametrize(
    "change",
    [
        {"ik_solver": "unknown"},
        {"joint_limits_rad": [[-2.8, 2.8]] * 14},
        {"plan_ttl_s": 30},
        {"base_frame": "world"},
        {"urdf_sha256": "0" * 64},
        {"max_joint_velocity_rad_per_s": -1},
    ],
)
def test_runtime_rejects_invalid_model_profiles(tmp_path, change):
    import json
    import shutil

    shutil.copytree(BUNDLE, tmp_path / "bundle")
    profile = tmp_path / "bundle/profiles/real-g1d/kinematics.json"
    profile.write_text(json.dumps({**json.loads(profile.read_text()), **change}))
    with pytest.raises(ValueError):
        G1DReadOnlyRuntime(tmp_path / "bundle", source=FakeLowStateSource())


def test_replayed_ticks_do_not_renew_freshness():
    now = [100.0]
    source = FakeLowStateSource()
    runtime = G1DReadOnlyRuntime(BUNDLE, source=source, clock=lambda: now[0])
    frame = make_lowstate_frame(mode_machine=7, tick=42, positions=[0.0] * 35)
    source.publish(frame, received_at=now[0])
    runtime.poll()
    now[0] += 0.11
    source.publish(frame, received_at=now[0])
    runtime.poll()
    assert runtime.state()["safety_gate"] == "state_stale"
