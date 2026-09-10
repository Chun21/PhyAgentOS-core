"""CycloneDDS wire contract for the external Dex1-1 serial service.

Unitree GO MotorCmds_/MotorStates_ layout; q=0 closed, q=5.4 open.
The deployed endpoint calibration must be verified before enabling this bridge.
This module never installs, starts, calibrates, or stops the external service.
"""

import math
import time
from dataclasses import dataclass, field

from cyclonedds import idl
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
from cyclonedds.core import Listener
from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import annotations, types
from cyclonedds.pub import DataWriter
from cyclonedds.qos import Policy, Qos
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

from PhyAgentOS.skill_runtime.g1d_dex1 import Dex1CommandSample, Dex1Error, validate_opening


@dataclass
@annotations.final
@annotations.autoid("sequential")
class DexMotorCmd(idl.IdlStruct, typename="unitree_go.msg.dds_.MotorCmd_"):
    mode: types.uint8
    q: types.float32
    dq: types.float32
    tau: types.float32
    kp: types.float32
    kd: types.float32
    reserve: types.array[types.uint32, 3]


@dataclass
@annotations.final
@annotations.autoid("sequential")
class DexMotorCmds(idl.IdlStruct, typename="unitree_go.msg.dds_.MotorCmds_"):
    cmds: types.sequence[DexMotorCmd] = field(default_factory=list)


@dataclass
@annotations.final
@annotations.autoid("sequential")
class DexMotorState(idl.IdlStruct, typename="unitree_go.msg.dds_.MotorState_"):
    mode: types.uint8
    q: types.float32
    dq: types.float32
    ddq: types.float32
    tau_est: types.float32
    q_raw: types.float32
    dq_raw: types.float32
    ddq_raw: types.float32
    temperature: types.uint8
    lost: types.uint32
    reserve: types.array[types.uint32, 2]


@dataclass
@annotations.final
@annotations.autoid("sequential")
class DexMotorStates(idl.IdlStruct, typename="unitree_go.msg.dds_.MotorStates_"):
    states: types.sequence[DexMotorState] = field(default_factory=list)


class CycloneDex1Bridge:
    """Independent side channels with strict normalized opening and source age."""

    def __init__(self, *, domain_id: int, verified_open_q: float, verified_closed_q: float):
        if (verified_open_q, verified_closed_q) != (5.4, 0.0):
            raise ValueError("unsupported or unverified Dex1 calibration")
        self._participant = DomainParticipant(domain_id)
        self._discovery = BuiltinDataReader(self._participant, BuiltinTopicDcpsPublication)
        self._publications = {}
        self._readers = {
            side: DataReader(
                self._participant,
                Topic(self._participant, f"rt/dex1/{side}/state", DexMotorStates),
                qos=Qos(Policy.History.KeepLast(1)),
            )
            for side in ("left", "right")
        }
        self._matched = {side: 0 for side in ("left", "right")}
        self._writers = {}
        for side in ("left", "right"):
            def matched(writer, status, selected=side):
                self._matched[selected] = status.current_count

            self._writers[side] = DataWriter(
                self._participant, Topic(self._participant, f"rt/dex1/{side}/cmd", DexMotorCmds),
                listener=Listener(on_publication_matched=matched))

    def read(self, side: str) -> tuple[float, float] | None:
        samples = self._readers[side].take(1)
        if not samples:
            return None
        sample = samples[0]
        if not sample.sample_info.valid_data or len(sample.states) != 1:
            raise Dex1Error("invalid Dex1 state frame")
        state = sample.states[0]
        if state.lost or not math.isfinite(state.q):
            raise Dex1Error("Dex1 motor communication fault")
        age = max(0.0, time.time() - sample.sample_info.source_timestamp / 1e9)
        return validate_opening(state.q / 5.4), time.monotonic() - age

    def write(self, sample: Dex1CommandSample) -> None:
        opening = validate_opening(sample.command.opening)
        self._writers[sample.command.side].write(
            DexMotorCmds([DexMotorCmd(1, opening * 5.4, 0.0, 0.0, 5.0, 0.05, [0, 0, 0])])
        )

    def require_exclusive(self, side):
        for item in self._discovery.take(1024):
            key = str(item.key)
            if item.sample_info.valid_data:
                self._publications[key] = item.topic_name
            else:
                self._publications.pop(key, None)
        own = str(self._writers[side].guid)
        topic = f"rt/dex1/{side}/cmd"
        if any(key != own and name == topic for key, name in self._publications.items()):
            raise Dex1Error(f"competing Dex1 {side} command writer")
        if not self._matched[side]:
            raise Dex1Error(f"Dex1 {side} command receiver unavailable")


class G1DCommandSink:
    """Route complete HG frames and optional external Dex1 commands explicitly."""

    def __init__(self, arm, dex1: CycloneDex1Bridge | None = None):
        self.arm, self.dex1 = arm, dex1

    def write(self, sample):
        if isinstance(sample, Dex1CommandSample):
            if self.dex1 is None:
                raise Dex1Error("requested Dex1 writer is unavailable")
            self.dex1.write(sample)
        else:
            self.arm.write(sample)
