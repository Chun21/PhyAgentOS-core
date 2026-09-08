import json
import socket
import threading
import time

import pytest

from PhyAgentOS.skill_runtime.g1d_adapter import SafetyFaultError
from PhyAgentOS.skill_runtime.g1d_authority import SupervisorAuthority


def test_supervisor_lease_requires_every_gate_and_expires(tmp_path):
    path = tmp_path / "supervisor.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    server.listen()
    server.settimeout(0.05)
    stop = threading.Event()
    approved = [False]

    def serve():
        while not stop.is_set():
            try:
                client, _ = server.accept()
            except socket.timeout:
                continue
            with client:
                request = json.loads(client.recv(4096))
                client.sendall(
                    (
                        json.dumps(
                            {
                                **request,
                                "exclusive": True,
                                "handoff_approved": approved[0],
                                "operator_ready": True,
                                "constraints_verified": True,
                            }
                        )
                        + "\n"
                    ).encode()
                )

    worker = threading.Thread(target=serve)
    worker.start()
    authority = SupervisorAuthority(path, "verified-digest")
    authority.start()
    try:
        time.sleep(0.03)
        with pytest.raises(SafetyFaultError):
            authority.require()
        approved[0] = True
        deadline = time.monotonic() + 1
        while True:
            try:
                authority.require()
                break
            except SafetyFaultError:
                assert time.monotonic() < deadline
                time.sleep(0.01)
        approved[0] = False
        time.sleep(0.08)
        with pytest.raises(SafetyFaultError):
            authority.require()
    finally:
        authority.close()
        stop.set()
        worker.join(timeout=1)
        server.close()
