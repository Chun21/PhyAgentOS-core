"""Verify CLI liveness, lost clients, and time-independent control authority."""

import time
from pathlib import Path

import pytest
from test_g1d_control_session import make_session, wait_for

from PhyAgentOS.skill_runtime.agent_lease import AgentControlLease, lease_deadline
from PhyAgentOS.skill_runtime.g1d_adapter import SafetyFaultError


def test_cli_heartbeat_runs_while_caller_is_idle_and_revokes_on_error(tmp_path, monkeypatch):
    monkeypatch.setattr("PhyAgentOS.skill_runtime.agent_lease.HEARTBEAT_INTERVAL_S", .01)
    lease = AgentControlLease(tmp_path)
    with pytest.raises(RuntimeError, match="CLI failed"):
        with lease:
            initial = lease.path.read_text()
            wait_for(lambda: lease.path.read_text() != initial)
            assert lease_deadline(lease.path, time.monotonic()) > time.monotonic()
            raise RuntimeError("CLI failed")
    assert not lease.path.exists()
    assert not lease._thread.is_alive()


@pytest.mark.parametrize("timestamp", ["nan", "inf", "garbage", "111", "90"])
def test_invalid_future_and_expired_heartbeats_are_rejected(tmp_path, timestamp):
    path = tmp_path / "lease"
    path.write_text(timestamp)
    with pytest.raises(ValueError):
        lease_deadline(path, 100)


def test_live_lease_supports_hours_without_resetting_state_watchdog(tmp_path):
    session = make_session(tmp_path)
    session.agent_lease = tmp_path / "lease"
    now = [100.0]
    session.clock = lambda: now[0]
    session._ready = True
    for elapsed in (0, 299, 301, 1801, 86400):
        now[0] = 100 + elapsed
        session.agent_lease.write_text(str(now[0]))
        session._monitor_at = now[0]
        session._renew_agent_lease()
        session.require()
    session._monitor_at -= 1
    with pytest.raises(SafetyFaultError, match="monitor expired"):
        session.require()


def test_revoked_lease_stops_writer_and_restores_mode(tmp_path):
    session = make_session(tmp_path)
    with AgentControlLease(tmp_path / "leases") as lease:
        session.agent_lease = lease.path
        try:
            session.start()
            wait_for(lambda: session.sent > 0)
            lease.close()
            wait_for(lambda: session.recovery == "ai_confirmed")
            assert not session.status()["ready"]
            assert "lease unavailable" in session.error
            count = session.sent
            time.sleep(.03)
            assert session.sent == count
        finally:
            session.close()


def test_expired_agent_cannot_take_control(tmp_path):
    session = make_session(tmp_path)
    session.agent_lease = tmp_path / "missing"
    with pytest.raises(SafetyFaultError, match="lease unavailable"):
        session.start()
    assert session.motion.releases == 0
    assert not session.sink.samples


def test_crashed_client_stops_control_and_cannot_be_revived(tmp_path):
    session = make_session(tmp_path)
    session.agent_lease = tmp_path / "lease"
    session.agent_lease.write_text(str(time.monotonic()))
    try:
        session.start()
        # A crashed client leaves its last timestamp instead of removing the file.
        session.agent_lease.write_text(str(time.monotonic() - 11))
        wait_for(lambda: session.recovery == "ai_confirmed")
        assert "heartbeat expired" in session.error
        session.agent_lease.write_text(str(time.monotonic()))
        with pytest.raises(SafetyFaultError):
            session.require()
    finally:
        session.close()


def test_second_cli_cannot_adopt_another_physical_session(tmp_path, monkeypatch):
    from types import SimpleNamespace

    from PhyAgentOS.skill_runtime.manager import RuntimeManager, RuntimeManagerError
    from PhyAgentOS.skill_runtime.manifest import load_manifest
    from PhyAgentOS.skill_runtime.state import RuntimeState, RuntimeStateStore

    manifest = load_manifest(Path(__file__).parents[1] / "bundles/g1d-manipulation/skill.yaml")
    store = RuntimeStateStore(tmp_path / "state")
    state = RuntimeState(skill_name=manifest.name, profile="real-g1d", status="running",
                         flow_name="existing", gateway_url=manifest.gateway_url)
    store.save(state)
    manager = RuntimeManager(catalog=SimpleNamespace(get=lambda _: manifest),
                             state_store=store, runtime_root=tmp_path / "runtime")
    monkeypatch.setattr(manager, "status", lambda _: SimpleNamespace(ready=True, state=state))
    with AgentControlLease(tmp_path / "leases") as lease:
        with pytest.raises(RuntimeManagerError, match="another session"):
            manager.start(manifest.name, "real-g1d", operator_confirmed=True, agent_lease=lease.path)
    assert store.load(manifest.name) == state


def test_cli_wrapper_revokes_lease_on_all_exit_paths(tmp_path, monkeypatch):
    from PhyAgentOS.cli import commands
    from PhyAgentOS.config import paths

    monkeypatch.setattr(paths, "get_config_path", lambda: tmp_path / "config.json")
    monkeypatch.setattr(commands, "_load_command_config", lambda *_: object())
    captured = []

    def run(*args, **kwargs):
        lease = kwargs["control_lease"]
        captured.append(lease.path)
        assert lease.path.exists()
        raise SystemExit(0)

    monkeypatch.setattr(commands, "_run_agent", run)
    with pytest.raises(SystemExit):
        commands.agent(message=None, session_id="cli:test", workspace=None, config=None,
                       markdown=False, logs=False, physical=True)
    assert captured and not Path(captured[0]).exists()
