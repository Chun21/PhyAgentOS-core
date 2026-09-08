"""Forge Action lifecycle binding for the G1_D executor."""

from __future__ import annotations

from typing import Any

from forge_tool import (
    ToolAccepted,
    ToolControlResponse,
    ToolEndpointError,
    ToolError,
    ToolEvent,
    ToolExecutionStatus,
    ToolResult,
    ToolResultResponse,
)

from PhyAgentOS.skill_runtime.g1d_adapter import AdapterError
from PhyAgentOS.skill_runtime.g1d_executor import ExecutorError, G1DExecutor
from PhyAgentOS.skill_runtime.g1d_planner import PlannerError


class G1DActionEndpoint:
    """Bind Gateway identities to one durable physical invocation, including retries."""

    def __init__(self, executor: G1DExecutor, validate: Any) -> None:
        self.executor = executor
        self.validate = validate
        self._aliases: dict[Any, str] = {}
        self._stops: dict[Any, str] = {}
        self._events: dict[Any, Any] = {}

    async def start(self, request: Any, context: Any, events: Any) -> ToolAccepted:
        self.validate(request, context)
        arguments = dict(request.arguments)
        key = context.execution_key
        if context.operation == "stop":
            outcome = self.executor.stop(invocation_id=arguments.get("invocation_id"))
            self._stops[key] = outcome
            self._events[key] = events
            return ToolAccepted(details={"status": outcome})
        if context.caller_id is not None and context.caller_id != arguments["caller_id"]:
            raise ToolEndpointError(
                ToolError("caller_mismatch", "Gateway caller differs from arguments")
            )
        try:
            record = self.executor.execute_pose(
                **arguments, invocation_id=key.invocation_id, attempt_id=key.attempt_id
            )
        except (ExecutorError, PlannerError, AdapterError, ValueError) as error:
            raise ToolEndpointError(ToolError("execution_rejected", str(error))) from error
        self._aliases[key] = record.invocation_id
        self._events[key] = events
        return ToolAccepted(details=record.as_result())

    def _record(self, key: Any) -> Any:
        identity = self._aliases.get(key, key.invocation_id)
        record = self.executor.get_invocation(identity)
        if key not in self._aliases and record.attempt_id != key.attempt_id:
            raise ExecutorError("attempt identity mismatch")
        return record

    async def cancel(self, key: Any, reason: str | None = None) -> ToolControlResponse:
        try:
            record = self._record(key)
        except ExecutorError as lookup_error:
            return ToolControlResponse(
                "cancel", "rejected", error=ToolError("not_found", str(lookup_error))
            )
        if record.status.is_terminal:
            return ToolControlResponse(
                "cancel", "terminal", details={"outcome": record.status.value}
            )
        self.executor.stop(invocation_id=record.invocation_id)
        return ToolControlResponse("cancel", "accepted")

    async def status(self, key: Any) -> ToolExecutionStatus:
        if key in self._stops:
            return ToolExecutionStatus("completed", details={"status": self._stops[key]})
        try:
            record = self._record(key)
        except ExecutorError as lookup_error:
            return ToolExecutionStatus("unknown", error=ToolError("not_found", str(lookup_error)))
        phase = {
            "pending": "accepted",
            "succeeded": "completed",
            "deadline_exceeded": "failed",
        }.get(record.status.value, record.status.value)
        error = (
            ToolError(record.status.value, record.stop_reason or record.status.value)
            if phase in ("failed", "unknown")
            else None
        )
        return ToolExecutionStatus(
            phase,
            error=error,
            details={
                "outcome": record.status.value,
                **record.as_result(),
                "evidence_url": f"/g1d/evidence/{record.invocation_id}",
            },
        )

    async def result(self, key: Any) -> ToolResultResponse:
        if key in self._stops:
            return ToolResultResponse(
                "available", ToolResult("succeeded", outputs={"status": self._stops[key]})
            )
        try:
            record = self._record(key)
        except ExecutorError:
            return ToolResultResponse("not_found")
        if not record.status.is_terminal:
            return ToolResultResponse("pending")
        status = "failed" if record.status.value == "deadline_exceeded" else record.status.value
        error = (
            ToolError(record.status.value, record.stop_reason or status)
            if status in ("failed", "unknown")
            else None
        )
        return ToolResultResponse(
            "available", ToolResult(status, outputs=record.as_result(), error=error)
        )

    async def emit_updates(self) -> None:
        for key, events in list(self._events.items()):
            response = await self.result(key)
            if response.status != "available":
                continue
            result = response.result
            assert result is not None
            event_type = {
                "succeeded": "executor_completed",
                "cancelled": "cancelled",
                "stopped": "stopped",
                "failed": "executor_failed",
            }.get(result.status)
            # Unknown has no terminal event in Forge; status/result reconciliation
            # carries its authoritative outcome without disguising it as failure.
            if event_type is not None:
                await events.emit(ToolEvent(event_type))
            del self._events[key]
