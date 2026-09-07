from __future__ import annotations

import pytest

from PhyAgentOS.skill_runtime.g1d_adapter import (
    ARM_SLOTS,
    TOTAL_MOTOR_SLOTS,
    CRCError,
    FakeLowStateSource,
    G1DAdapter,
    LowStateFrame,
    ModeLossError,
    SafetyFaultError,
    StaleStateError,
    make_lowstate_frame,
)


class Clock:
    def __init__(self) -> None:
        self.value = 100.0

    def __call__(self) -> float:
        return self.value


def _frame(*, mode_machine: int = 7, safety_ok: bool = True) -> LowStateFrame:
    return make_lowstate_frame(
        mode_machine=mode_machine,
        tick=42,
        positions=[float(index) for index in range(TOTAL_MOTOR_SLOTS)],
        mode=[index % 3 for index in range(TOTAL_MOTOR_SLOTS)],
        safety_ok=safety_ok,
    )


def test_valid_fake_lowstate_is_cached_and_query_exposes_state() -> None:
    clock = Clock()
    adapter = G1DAdapter(clock=clock, max_state_age_s=0.1, approved_mode_machine=7)

    adapter.ingest(_frame(), received_at=clock())
    state = adapter.query_state()

    assert state.mode_machine == 7
    assert state.frame == 42
    assert state.state_age_ms == 0
    assert state.safety_gate == "ready"
    assert tuple(item.q for item in state.left_arm) == tuple(float(i) for i in range(15, 22))
    assert tuple(item.q for item in state.right_arm) == tuple(float(i) for i in range(22, 29))
    assert state.dex1.available is False
    assert state.active_operation is None


def test_hold_frame_is_complete_preserves_non_arm_slots_and_mode() -> None:
    adapter = G1DAdapter(clock=lambda: 10.0, approved_mode_machine=7)
    adapter.ingest(_frame(), received_at=10.0)

    command = adapter.hold_frame()

    assert len(command.motor_cmd) == TOTAL_MOTOR_SLOTS
    assert command.mode_machine == 7
    assert tuple(item.q for item in command.motor_cmd) == tuple(float(i) for i in range(35))
    assert all(command.motor_cmd[index].q == float(index) for index in range(15))
    assert all(command.motor_cmd[index].q == float(index) for index in ARM_SLOTS)
    assert command.crc == command.calculated_crc()


def test_bad_crc_is_rejected_before_cache_changes() -> None:
    adapter = G1DAdapter(clock=lambda: 10.0, approved_mode_machine=7)
    valid = _frame()
    adapter.ingest(valid, received_at=10.0)
    broken = LowStateFrame.from_frame(valid, crc=valid.crc ^ 1)

    with pytest.raises(CRCError):
        adapter.ingest(broken, received_at=10.01)

    assert adapter.query_state().frame == valid.tick


def test_stale_state_mode_loss_and_safety_fault_are_explicit() -> None:
    clock = Clock()
    adapter = G1DAdapter(clock=clock, max_state_age_s=0.1, approved_mode_machine=7)
    adapter.ingest(_frame(), received_at=clock())
    clock.value += 0.101
    with pytest.raises(StaleStateError):
        adapter.require_ready()

    clock.value = 200.0
    adapter.ingest(_frame(mode_machine=0), received_at=clock())
    with pytest.raises(ModeLossError):
        adapter.require_ready()

    clock.value = 300.0
    adapter.ingest(_frame(safety_ok=False), received_at=clock())
    with pytest.raises(SafetyFaultError):
        adapter.require_ready()


def test_action_readiness_requires_an_explicit_approved_mode() -> None:
    adapter = G1DAdapter(clock=lambda: 10.0)
    adapter.ingest(_frame(), received_at=10.0)

    with pytest.raises(ModeLossError, match="not configured"):
        adapter.require_ready()


def test_fake_source_is_the_only_input_needed_by_poll() -> None:
    source = FakeLowStateSource()
    adapter = G1DAdapter(clock=lambda: 12.0, approved_mode_machine=7)
    source.publish(_frame(), received_at=12.0)

    adapter.poll(source)

    assert adapter.query_state().frame == 42


def test_fake_wire_bytes_are_decoded_before_crc_validation() -> None:
    adapter = G1DAdapter(clock=lambda: 12.0, approved_mode_machine=7)
    frame = _frame()

    adapter.ingest(frame.to_bytes(), received_at=12.0)

    assert adapter.query_state().mode_machine == frame.mode_machine


def test_active_operation_is_visible_in_state_query() -> None:
    adapter = G1DAdapter(clock=lambda: 12.0)
    adapter.ingest(_frame(), received_at=12.0)
    adapter.set_active_operation("invocation-1")

    assert adapter.query_state().active_operation == "invocation-1"
