"""Exercise native RPC matching on a loopback-only, non-robot DDS domain."""

import json
import threading
import time

import pytest


def test_motion_switcher_rpc_identity_status_and_timeout(monkeypatch):
    monkeypatch.setenv("CYCLONEDDS_URI", '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic

    from PhyAgentOS.skill_runtime.g1d_motion_switcher import (
        MotionSwitcher,
        Request,
        RequestIdentity,
        Response,
        ResponseHeader,
        ResponseStatus,
    )

    participant = DomainParticipant(203)
    reader = DataReader(participant, Topic(participant, "rt/api/motion_switcher/request", Request))
    writer = DataWriter(participant, Topic(participant, "rt/api/motion_switcher/response", Response))
    stop = threading.Event()
    mode = ["ai"]
    behavior = ["normal"]

    def serve():
        while not stop.wait(.002):
            for request in reader.take(32):
                if not request.sample_info.valid_data or behavior[0] == "silent":
                    continue
                identity = request.header.identity
                api = identity.api_id
                if api == 1003:
                    mode[0] = ""
                elif api == 1002:
                    mode[0] = json.loads(request.parameter)["name"]
                writer.write(Response(ResponseHeader(RequestIdentity(identity.id - 1, api),
                    ResponseStatus(0)), '{"name":"wrong_response"}', []))
                writer.write(Response(ResponseHeader(identity,
                    ResponseStatus(17 if behavior[0] == "error" else 0)),
                    json.dumps({"name": mode[0]}), []))

    server = threading.Thread(target=serve)
    server.start()
    try:
        client = MotionSwitcher(203, timeout=3)
        assert client.check() == "ai"
        client.release()
        assert client.check() == ""
        client.restore()
        assert client.check() == "ai"
        behavior[0] = "error"
        with pytest.raises(RuntimeError, match="code 17"):
            client.check()
        behavior[0] = "silent"
        client.timeout = .1
        started = time.monotonic()
        with pytest.raises(TimeoutError, match="outcome unknown"):
            client.check()
        assert time.monotonic() - started < .5
    finally:
        stop.set()
        server.join(2)
