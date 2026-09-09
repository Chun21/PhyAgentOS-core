"""Embed the upstream Forge Tool Gateway and handler in the robot-side node.

The Gateway mailbox carries the unchanged Forge envelopes in-process. Dora
supervises this node; DDS ingestion and model solving belong to the Skill.
Command publication requires an explicitly configured control authority.
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

from PhyAgentOS.skill_runtime.g1d_action_endpoint import G1DActionEndpoint
from PhyAgentOS.skill_runtime.g1d_bridge import CycloneLowStateSource
from PhyAgentOS.skill_runtime.g1d_execution_runtime import G1DExecutionRuntime
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
        return ToolControlResponse(
            "cancel", "rejected", error=ToolError("not_found", "No accepted operation")
        )

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
        self.action_endpoint = (
            G1DActionEndpoint(runtime.executor, ReadOnlyEndpoint(runtime).validate)
            if isinstance(runtime, G1DExecutionRuntime)
            else None
        )
        operations = tuple(
            ToolOperationDescriptor(
                name=spec["operation"],
                semantics=spec["semantics"],
                status_supported=spec["semantics"] == "action",
                cancellable=spec["semantics"] == "action",
                max_concurrency=spec["max_concurrency"],
            )
            for spec in runtime.tools.values()
        )
        self.descriptor = ToolEndpointDescriptor(TOOL_ENDPOINT_PROTOCOL, ENDPOINT, operations)
        self.handler = ToolEndpointHandler(
            self.descriptor,
            endpoint_instance_id=runtime.instance_id,
            operations={
                spec["operation"]: (
                    self.action_endpoint
                    if spec["semantics"] == "action" and self.action_endpoint
                    else ReadOnlyEndpoint(runtime)
                )
                for spec in runtime.tools.values()
            },
        )
        specs = [
            ToolSpec(
                tool_id=spec["tool_id"],
                implementation_id=("g1d.internal.execution.v1" if self.action_endpoint
                                   else "g1d.internal.readonly.v1"),
                endpoint_id=ENDPOINT,
                operation=spec["operation"],
                semantics=spec["semantics"],
                description=spec["description"],
                input_schema=spec["input_schema"],
                output_schema=spec["output_schema"],
                robot_frame_profile=RobotFrameProfile(
                    robot_id="unitree-g1d",
                    base_frame=runtime.config["base_frame"],
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

    async def _event(self, envelope: Any) -> None:
        self._receive(envelope)

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
        if isinstance(self.runtime, G1DExecutionRuntime):
            self.runtime.control.start()
        self.worker.start()

    def close(self) -> None:
        self.stop_event.set()
        self.worker.join(timeout=5)
        self.tool_gateway.close()
        if isinstance(self.runtime, G1DExecutionRuntime):
            self.runtime.control.close()
        if self.worker.is_alive():
            raise RuntimeError("G1_D endpoint did not stop")

    def _run(self) -> None:
        try:
            with asyncio.Runner() as runner:
                next_announce = 0.0
                while not self.stop_event.is_set():
                    if not isinstance(self.runtime, G1DExecutionRuntime):
                        self.runtime.poll()
                    elif self.runtime.control.error is not None:
                        raise RuntimeError(
                            "execution control loop failed"
                        ) from self.runtime.control.error
                    if time.monotonic() >= next_announce:
                        self._announce()
                        next_announce = time.monotonic() + 0.05
                    message = self.tool_gateway.take_outbound()
                    if message is not None and message.kind != "provider.registry_response":
                        for response in runner.run(
                            self.handler.dispatch(message.envelope, event_sink=self._event)
                        ):
                            self._receive(response)
                    if self.action_endpoint is not None:
                        runner.run(self.action_endpoint.emit_updates())
                    self.tool_gateway.sweep()
                    self.stop_event.wait(0.002)
        except BaseException as error:
            self.worker_error = error
            if isinstance(self.runtime, G1DExecutionRuntime):
                record = self.runtime.executor.active_invocation()
                if record is not None and not record.status.is_terminal:
                    self.runtime.executor.mark_unknown(
                        record.invocation_id, reason="gateway_transport_loss"
                    )
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
        if isinstance(self.runtime, G1DExecutionRuntime):
            executor = self.runtime.executor

            @app.get("/g1d/evidence/{invocation_id}")
            def evidence(invocation_id: str):
                from fastapi import HTTPException
                from fastapi.responses import StreamingResponse

                from PhyAgentOS.skill_runtime.g1d_executor import UnknownInvocationError

                try:
                    records = executor.evidence_json(invocation_id)
                except UnknownInvocationError as error:
                    raise HTTPException(404, str(error)) from error

                def chunks():
                    yield "["
                    for index, record in enumerate(records):
                        if index:
                            yield ","
                        for offset in range(0, len(record), 65536):
                            yield record[offset:offset + 65536]
                    yield "]"

                return StreamingResponse(chunks(), media_type="application/json")

        return app


def main() -> None:
    import uvicorn

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default=os.environ.get("PAOS_G1D_PROFILE", "real-g1d"))
    parser.add_argument("--host", default=os.environ.get("PAOS_G1D_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("PAOS_G1D_PORT", "19082")))
    parser.add_argument(
        "--control-profile", type=Path, help="site-verified 35-slot control profile"
    )
    parser.add_argument(
        "--authority-socket", type=Path, help="approved robot-supervisor Unix socket"
    )
    parser.add_argument("--journal", type=Path, help="durable invocation/evidence SQLite path")
    parser.add_argument("--operator-confirmed", action="store_true",
                        help="explicit supervised native control session; releases factory ai mode")
    parser.add_argument("--session-seconds", type=float, default=300,
                        help="bounded native operator session, 10..1800 seconds")
    parser.add_argument("--max-arm-excursion-rad", type=float,
                        help="native session joint excursion, capped at 1.5 rad")
    parser.add_argument("--controller-lock", type=Path,
                        default=Path.home() / ".phyagent/g1d-controller.lock")
    args = parser.parse_args()
    if os.environ.get("PAOS_G1D_CONTROL_ENABLED") == "1" and not args.operator_confirmed:
        args.operator_confirmed = True
        args.control_profile = Path(os.environ["PAOS_G1D_CONTROL_PROFILE"])
        args.journal = Path(os.environ["PAOS_G1D_JOURNAL"])
        args.max_arm_excursion_rad = float(os.environ.get("PAOS_G1D_MAX_ARM_EXCURSION_RAD", "1.5"))
    if args.operator_confirmed and args.authority_socket:
        parser.error("choose native operator session or external authority, not both")
    if args.max_arm_excursion_rad is not None and not args.operator_confirmed:
        parser.error("excursion override requires operator-confirmed native control")
    if any((args.control_profile, args.authority_socket, args.journal, args.operator_confirmed)):
        if not (args.control_profile and args.journal and (args.authority_socket or args.operator_confirmed)):
            parser.error("execution requires control-profile, journal and operator-confirmed or authority-socket")
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
    source: Any = CycloneLowStateSource(domain_id=int(os.environ.get("PAOS_G1D_DDS_DOMAIN", "0")))
    shared_source = None
    if args.operator_confirmed:
        from PhyAgentOS.skill_runtime.g1d_bridge import SharedLowStateSource

        shared_source = SharedLowStateSource(source)
        source = shared_source
    authority = None
    session = None
    runtime: G1DReadOnlyRuntime
    if args.control_profile:
        import json

        from PhyAgentOS.skill_runtime.g1d_authority import SupervisorAuthority
        from PhyAgentOS.skill_runtime.g1d_bridge import CycloneLowCmdSink
        from PhyAgentOS.skill_runtime.g1d_dex1 import Dex1Integration
        from PhyAgentOS.skill_runtime.g1d_dex1_bridge import CycloneDex1Bridge, G1DCommandSink
        from PhyAgentOS.skill_runtime.g1d_planner import digest_json

        config = json.loads(args.control_profile.read_text())
        if args.max_arm_excursion_rad is not None:
            config["max_arm_excursion_rad"] = args.max_arm_excursion_rad
        domain = int(os.environ.get("PAOS_G1D_DDS_DOMAIN", "0"))
        if args.operator_confirmed and config.get("dex1"):
            parser.error("native arm-only session does not enable Dex1 commands")
        dex_bridge = (
            CycloneDex1Bridge(domain_id=domain, **config["dex1"]) if config.get("dex1") else None
        )
        dex1 = (
            Dex1Integration(
                clock=time.monotonic, sources={side: dex_bridge for side in ("left", "right")}
            )
            if dex_bridge
            else None
        )
        sink: Any = G1DCommandSink(CycloneLowCmdSink(domain_id=domain), dex_bridge)
        if args.operator_confirmed:
            from PhyAgentOS.skill_runtime.g1d_control_session import ControlSession, LowCmdDiscovery
            from PhyAgentOS.skill_runtime.g1d_motion_switcher import MotionSwitcher

            session = ControlSession(
                config=config, source=source,
                sink=sink.arm, motion=MotionSwitcher(domain), discovery=LowCmdDiscovery(domain),
                lock_path=args.controller_lock, duration_s=args.session_seconds)
            sink = session
            ownership = session.require
        else:
            authority = SupervisorAuthority(args.authority_socket, digest_json(config))
            ownership = authority.require
        args.journal.parent.mkdir(parents=True, exist_ok=True)
        runtime = G1DExecutionRuntime(
            root,
            source=source,
            sink=sink,
            journal_path=args.journal,
            approved_mode=config["approved_mode"],
            gains=config["gains"],
            modes=config["modes"],
            require_ownership=ownership,
            clock=time.monotonic,
            dex1=dex1,
        )
        if authority is not None:
            authority.start()
    else:
        runtime = G1DReadOnlyRuntime(root, source=source, profile=args.profile)
    gateway = G1DGateway(runtime)
    server = uvicorn.Server(uvicorn.Config(gateway.app(), host=args.host, port=args.port))
    finished = threading.Event()

    def monitor_session():
        while not finished.wait(.1):
            if session.error or session._stop.is_set() or gateway.worker_error or runtime.control.error:
                server.should_exit = True
                return

    def monitor_dora() -> None:
        assert dora_node is not None
        while not finished.is_set():
            event = dora_node.next(timeout=0.1)
            if event is None or event["type"] == "STOP":
                server.should_exit = True
                return

    monitors = []
    if dora_node is not None:
        monitor = threading.Thread(target=monitor_dora, name="g1d-dora-lifecycle", daemon=True)
        monitor.start()
        monitors.append(monitor)
    try:
        if shared_source is not None:
            shared_source.start()
        if session is not None:
            session.start()
            monitor = threading.Thread(target=monitor_session, name="g1d-session-lifecycle", daemon=True)
            monitor.start()
            monitors.append(monitor)
        server.run()
    finally:
        finished.set()
        if authority is not None:
            authority.close()
        if session is not None:
            session.close()
            print("g1d_control_session", json.dumps(session.status()), flush=True)
        for monitor in monitors:
            monitor.join(timeout=2)
        if isinstance(runtime, G1DExecutionRuntime):
            runtime.executor.close()
        if shared_source is not None:
            shared_source.close()
