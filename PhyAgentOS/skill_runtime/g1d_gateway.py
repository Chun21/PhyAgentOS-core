"""Embed the upstream Forge Tool Gateway and handler in the read-only node.

The Gateway mailbox carries the unchanged Forge envelopes in-process. Dora
supervises this node; DDS ingestion and model solving belong to the Skill.
No command publisher or control-mode client is constructed.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI
from forge_gateway.config import ToolGatewayConfig, ToolProviderRouteConfig
from forge_gateway.controllers.tool_controller import register_tool_routes
from forge_gateway.domain.tool_catalog import RobotFrameProfile, ToolSpec
from forge_gateway.services.tool_gateway_service import ToolGatewayService
from forge_tool import (
    TOOL_ENDPOINT_PROTOCOL,
    EndpointStatus,
    ToolAccepted,
    ToolControlResponse,
    ToolEndpointDescriptor,
    ToolEndpointError,
    ToolError,
    ToolExecutionStatus,
    ToolOperationDescriptor,
    ToolResult,
    ToolResultResponse,
    make_endpoint_status_envelope,
    make_registration_envelope,
)
from forge_tool.handler import ToolEndpointHandler
from jsonschema import Draft202012Validator, ValidationError

from PhyAgentOS.skill_runtime.g1d_bridge import CycloneLowStateSource
from PhyAgentOS.skill_runtime.g1d_planner import PlannerError
from PhyAgentOS.skill_runtime.g1d_runtime import G1DReadOnlyRuntime

ENDPOINT = "g1d.dual_arm"
PROVIDER_INPUT = "g1d/to_gateway"


class ReadOnlyEndpoint:
    def __init__(self, runtime: G1DReadOnlyRuntime) -> None:
        self.runtime = runtime

    def validate(self, request: Any, context: Any) -> None:
        schema = self.runtime.tools[f"{ENDPOINT}.{context.operation}"]["input_schema"]
        try:
            Draft202012Validator(schema).validate(dict(request.arguments))
        except ValidationError as error:
            raise ToolEndpointError(ToolError("invalid_arguments", error.message)) from error

    async def query(self, request: Any, context: Any) -> ToolResult:
        self.validate(request, context)
        try:
            outputs = (
                self.runtime.state()
                if context.operation == "state"
                else self.runtime.plan_pose(dict(request.arguments))
            )
            return ToolResult("succeeded", outputs=outputs)
        except (PlannerError, ValueError, TypeError) as error:
            return ToolResult("failed", error=ToolError("plan_rejected", str(error)))

    async def start(self, request: Any, context: Any, events: Any) -> ToolAccepted:
        self.validate(request, context)
        if context.operation == "execute_pose":
            raise ToolEndpointError(
                ToolError(
                    "action_not_ready", "Execution slice unavailable; no command was published"
                )
            )
        # No execution has ever been admitted by this Runtime. Reject the Action
        # before admission with an explicit, repeatable no-active-operation outcome.
        raise ToolEndpointError(
            ToolError(
                "no_active_operation", "No active operation", details={"status": "already_stopped"}
            )
        )

    async def cancel(self, key: Any, reason: str | None = None) -> ToolControlResponse:
        return ToolControlResponse("cancel", "not_found")

    async def status(self, key: Any) -> ToolExecutionStatus:
        return ToolExecutionStatus(
            "unknown", error=ToolError("no_active_operation", "No accepted operation")
        )

    async def result(self, key: Any) -> ToolResultResponse:
        return ToolResultResponse("not_found")


class G1DGateway:
    """Lifecycle owner for the official Gateway and its embedded endpoint."""

    def __init__(self, runtime: G1DReadOnlyRuntime) -> None:
        self.runtime = runtime
        operations = tuple(
            ToolOperationDescriptor(
                name=spec["operation"],
                semantics=spec["semantics"],
                status_supported=spec["semantics"] == "action",
                max_concurrency=spec["max_concurrency"],
            )
            for spec in runtime.tools.values()
        )
        self.descriptor = ToolEndpointDescriptor(TOOL_ENDPOINT_PROTOCOL, ENDPOINT, operations)
        self.handler = ToolEndpointHandler(
            self.descriptor,
            endpoint_instance_id=runtime.instance_id,
            operations={
                spec["operation"]: ReadOnlyEndpoint(runtime) for spec in runtime.tools.values()
            },
        )
        specs = [
            ToolSpec(
                tool_id=spec["tool_id"],
                implementation_id="g1d.internal.readonly.v1",
                endpoint_id=ENDPOINT,
                operation=spec["operation"],
                semantics=spec["semantics"],
                description=spec["description"],
                input_schema=spec["input_schema"],
                output_schema=spec["output_schema"],
                robot_frame_profile=RobotFrameProfile(
                    robot_id="unitree-g1d",
                    base_frame="g1d_base",
                    tool_frame="tcp",
                    frames={"base_link": "pelvis", "left_tcp": "L_ee", "right_tcp": "R_ee"},
                ),
            )
            for spec in runtime.tools.values()
        ]
        self.tool_gateway = ToolGatewayService(
            ToolGatewayConfig(
                enabled=True,
                invoke_timeout_ms=10000,
                providers=[ToolProviderRouteConfig(ENDPOINT, PROVIDER_INPUT, "gateway/to_g1d")],
                specs=specs,
            )
        )
        self.stop_event = threading.Event()
        self.worker_error: BaseException | None = None
        self.worker = threading.Thread(target=self._run, name="g1d-tool-endpoint", daemon=True)

    def readiness(self) -> dict[str, Any]:
        return self.runtime.state()

    def _receive(self, envelope: Any) -> None:
        self.tool_gateway.handle_input(PROVIDER_INPUT, envelope, received_at=time.monotonic())

    def _announce(self) -> None:
        self._receive(
            make_registration_envelope(
                self.descriptor,
                endpoint_instance_id=self.runtime.instance_id,
                request_id="g1d-registration",
            )
        )
        self._receive(
            make_endpoint_status_envelope(
                EndpointStatus(ENDPOINT, "ready", details=self.runtime.state()),
                endpoint_instance_id=self.runtime.instance_id,
            )
        )

    def start(self) -> None:
        self._announce()
        self.worker.start()

    def close(self) -> None:
        self.stop_event.set()
        self.worker.join(timeout=5)
        self.tool_gateway.close()
        if self.worker.is_alive():
            raise RuntimeError("G1_D endpoint did not stop")

    def _run(self) -> None:
        try:
            with asyncio.Runner() as runner:
                next_announce = 0.0
                while not self.stop_event.is_set():
                    self.runtime.poll()
                    if time.monotonic() >= next_announce:
                        self._announce()
                        next_announce = time.monotonic() + 0.05
                    message = self.tool_gateway.take_outbound()
                    if message is not None and message.kind != "provider.registry_response":
                        for response in runner.run(self.handler.dispatch(message.envelope)):
                            self._receive(response)
                    self.tool_gateway.sweep()
                    self.stop_event.wait(0.002)
        except BaseException as error:
            self.worker_error = error
            self.tool_gateway.close()

    def app(self) -> FastAPI:
        @asynccontextmanager
        async def lifespan(app: FastAPI):
            self.start()
            try:
                yield
            finally:
                self.close()

        app = FastAPI(lifespan=lifespan)
        register_tool_routes(app, self)
        return app


def main() -> None:
    import uvicorn

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default=os.environ.get("PAOS_G1D_PROFILE", "real-g1d"))
    parser.add_argument("--host", default=os.environ.get("PAOS_G1D_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("PAOS_G1D_PORT", "19082")))
    args = parser.parse_args()
    if args.profile != "real-g1d":
        parser.error("unknown profile")
    # Establish the managed Dora node connection only when launched by Dora.
    # The embedded Gateway carrier does not require dataflow request ports.
    dora_node = None
    if "DORA_NODE_CONFIG" in os.environ:
        from dora import Node

        dora_node = Node()
    root = Path(
        os.environ.get("PAOS_SKILL_ROOT", str(Path(__file__).resolve().parents[2] / "bundle"))
    )
    runtime = G1DReadOnlyRuntime(
        root,
        source=CycloneLowStateSource(domain_id=int(os.environ.get("PAOS_G1D_DDS_DOMAIN", "0"))),
        profile=args.profile,
    )
    server = uvicorn.Server(
        uvicorn.Config(G1DGateway(runtime).app(), host=args.host, port=args.port)
    )
    finished = threading.Event()

    def monitor_dora() -> None:
        assert dora_node is not None
        while not finished.is_set():
            event = dora_node.next(timeout=0.1)
            if event is None or event["type"] == "STOP":
                server.should_exit = True
                return

    monitor = None
    if dora_node is not None:
        monitor = threading.Thread(target=monitor_dora, name="g1d-dora-lifecycle", daemon=True)
        monitor.start()
    try:
        server.run()
    finally:
        finished.set()
        if monitor is not None:
            monitor.join(timeout=2)
