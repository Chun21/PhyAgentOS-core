"""Bounded Unitree MotionSwitcher RPC over DDS, without a project dependency."""

import json
import threading
import time
from dataclasses import dataclass

from cyclonedds import idl
from cyclonedds.core import Listener
from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import annotations as annotate
from cyclonedds.idl import types
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic


@dataclass
@annotate.final
@annotate.autoid("sequential")
class RequestIdentity(idl.IdlStruct, typename="unitree_api.msg.dds_.RequestIdentity_"):
    id: types.int64
    api_id: types.int64


@dataclass
@annotate.final
@annotate.autoid("sequential")
class RequestLease(idl.IdlStruct, typename="unitree_api.msg.dds_.RequestLease_"):
    id: types.int64


@dataclass
@annotate.final
@annotate.autoid("sequential")
class RequestPolicy(idl.IdlStruct, typename="unitree_api.msg.dds_.RequestPolicy_"):
    priority: types.int32
    noreply: bool


@dataclass
@annotate.final
@annotate.autoid("sequential")
class RequestHeader(idl.IdlStruct, typename="unitree_api.msg.dds_.RequestHeader_"):
    identity: RequestIdentity
    lease: RequestLease
    policy: RequestPolicy


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Request(idl.IdlStruct, typename="unitree_api.msg.dds_.Request_"):
    header: RequestHeader
    parameter: str
    binary: types.sequence[types.uint8]


@dataclass
@annotate.final
@annotate.autoid("sequential")
class ResponseStatus(idl.IdlStruct, typename="unitree_api.msg.dds_.ResponseStatus_"):
    code: types.int32


@dataclass
@annotate.final
@annotate.autoid("sequential")
class ResponseHeader(idl.IdlStruct, typename="unitree_api.msg.dds_.ResponseHeader_"):
    identity: RequestIdentity
    status: ResponseStatus


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Response(idl.IdlStruct, typename="unitree_api.msg.dds_.Response_"):
    header: ResponseHeader
    data: str
    binary: types.sequence[types.uint8]


class MotionSwitcher:
    def __init__(self, domain_id=0, timeout=2.0):
        self.participant = DomainParticipant(domain_id)
        self.response_matched = 0
        self.reader = DataReader(self.participant, Topic(
            self.participant, "rt/api/motion_switcher/response", Response),
            listener=Listener(on_subscription_matched=self._response_matched))
        self.matched = 0
        self.writer = DataWriter(self.participant, Topic(
            self.participant, "rt/api/motion_switcher/request", Request),
            listener=Listener(on_publication_matched=self._matched))
        self.timeout = timeout
        self.lock = threading.Lock()

    def _matched(self, writer, status):
        self.matched = status.current_count

    def _response_matched(self, reader, status):
        self.response_matched = status.current_count

    def call(self, api, parameters):
        with self.lock:
            deadline = time.monotonic() + self.timeout
            while not self.matched or not self.response_matched:
                if time.monotonic() >= deadline:
                    raise TimeoutError("MotionSwitcher request reader not discovered")
                time.sleep(.01)
            identity = time.monotonic_ns()
            self.writer.write(Request(RequestHeader(RequestIdentity(identity, api),
                RequestLease(0), RequestPolicy(0, False)), json.dumps(parameters), []))
            while time.monotonic() < deadline:
                for response in self.reader.take(32):
                    if not response.sample_info.valid_data:
                        continue
                    if response.header.identity.id != identity:
                        continue
                    if response.header.identity.api_id != api:
                        raise RuntimeError("MotionSwitcher API mismatch")
                    if response.header.status.code != 0:
                        raise RuntimeError(f"MotionSwitcher API {api}: code {response.header.status.code}")
                    return response.data
                time.sleep(.002)
            raise TimeoutError(f"MotionSwitcher API {api} response timeout; outcome unknown")

    def check(self):
        value = json.loads(self.call(1001, {}))
        if not isinstance(value, dict) or not isinstance(value.get("name"), str):
            raise RuntimeError("invalid MotionSwitcher mode response")
        return value["name"]

    def release(self):
        self.call(1003, {})

    def restore(self):
        self.call(1002, {"name": "ai"})
