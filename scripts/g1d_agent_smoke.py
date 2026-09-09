"""Supervised 1 cm out-and-back through Skill activation and AgentTaskCoordinator."""

from __future__ import annotations

import argparse
import asyncio
import copy
import json
import math
import sys
from pathlib import Path
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PhyAgentOS.agent.experience.activation import SkillActivationManager
from PhyAgentOS.agent.experience.store import ExperienceStore
from PhyAgentOS.agent.skills import SkillsLoader
from PhyAgentOS.config.schema import ForgeConfig
from PhyAgentOS.forge.binding import ForgeSkillBindingResolver
from PhyAgentOS.forge.task import AgentTaskCoordinator
from PhyAgentOS.forge.tool_client import ForgeToolClient
from PhyAgentOS.skill_runtime.catalog import SkillCatalog
from PhyAgentOS.skill_runtime.integration import ActiveRuntimeRegistry, ActiveSkillRuntime
from PhyAgentOS.verification.contracts import TaskVerificationContract

SKILL = "g1d-manipulation"
PREFIX = "g1d.dual_arm."


def outputs(response):
    data = response["data"]
    result = data.get("response", data).get("result", data)
    if result.get("status") != "succeeded":
        raise RuntimeError(f"Query failed: {response}")
    return result["outputs"]


async def execute(coordinator, task_id, target, observations=None):
    operation_seconds = 45 if target.get("gesture") == "wave" else 15
    plan = outputs(await coordinator.invoke_query(task_id, PREFIX + "plan_pose", target))
    accepted = await coordinator.start_action(task_id, PREFIX + "execute_pose", {
        "plan_id": plan["plan_id"], "operation_deadline_s": operation_seconds,
    })
    invocation = accepted["data"]["invocation_id"]
    deadline = asyncio.get_running_loop().time() + operation_seconds + 5
    next_observation = 0.0
    while True:
        result = await coordinator.client.invocation_result(invocation)
        coordinator.observe_action(task_id, invocation, result)
        now = asyncio.get_running_loop().time()
        if observations is not None and now >= next_observation:
            observations.append(outputs(await coordinator.invoke_query(task_id, PREFIX + "state", {})))
            next_observation = now + .3
        if result["data"]["status"] == "available":
            if result["data"]["result"]["status"] != "succeeded":
                raise RuntimeError(f"Physical Action did not succeed: {result}")
            return result
        if asyncio.get_running_loop().time() > deadline:
            cancel = await coordinator.client.cancel_invocation(invocation)
            coordinator.record_cancel_response(task_id, invocation, cancel)
            raise RuntimeError("Action deadline exceeded; cancellation requested, outcome unresolved")
        await asyncio.sleep(.1)


async def run(args):
    workspace = args.workspace.resolve()
    workspace.mkdir(parents=True, exist_ok=True)
    catalog = SkillCatalog(args.skills_root)
    manifest = catalog.get(SKILL)
    async with ForgeToolClient(args.gateway) as client:
        state = outputs(await client.invoke_query_tool(PREFIX + "state", {}))
        print(json.dumps(state, ensure_ascii=False), flush=True)
        if not args.execute:
            return
        if not state.get("action_ready"):
            raise RuntimeError("Start a supervised native control session before execution")
        active = ActiveSkillRuntime(
            skill_name=SKILL, skill_version=state["skill_version"], profile="real-g1d",
            runtime_instance_id=state["runtime_instance_id"], gateway_url=args.gateway,
            gateway_identity=None, client=client,
            invocation_ids=set(), session_ids=set(), task_binding_ids=set(),
        )
        registry = ActiveRuntimeRegistry(active)
        binding = ForgeSkillBindingResolver(registry, catalog=catalog)
        activation = SkillActivationManager(workspace=workspace, store=ExperienceStore(workspace),
            runtime_availability_provider=registry.is_available, binding_resolver=binding)
        activation.skills = SkillsLoader(workspace, installed_skills_dir=catalog.root,
            runtime_availability_provider=registry.is_available)
        coordinator = AgentTaskCoordinator(workspace=workspace,
            config=ForgeConfig(evidence={"capture_timeout_s": .2, "post_capture_timeout_s": .2}),
            client=client, binding_resolver=binding, activation_manager=activation,
            runtime_invocation_ids=active.invocation_ids, runtime_session_ids=active.session_ids,
            runtime_task_binding_ids=active.task_binding_ids)
        session_key = "supervised-g1d-" + uuid4().hex
        return_report = getattr(args, "return_report", None)
        wave = getattr(args, "wave", False)
        wave_arm = getattr(args, "wave_arm", "right")
        description = (f"Continuous {wave_arm}-arm greeting and return" if wave else
                       "Return both end effectors to recorded pre-motion poses" if return_report
                       else "Move both unloaded end effectors 1 cm along fixed-base X and return")
        activation.begin_turn(session_key, description)
        selected, _, _ = await activation.activate(session_key=session_key, name=manifest.name,
            role="primary")
        task = await coordinator.create_task(task_description=description,
            verification=TaskVerificationContract(mode="off"),
            activation_id=selected.activation_id, origin_session_key=session_key)
        report = {"task_id": task.task_id, "accepted": False, "scope": "robot_state_only"}
        report_path = workspace / f"{task.task_id}-motion.json"
        try:
            before = outputs(await coordinator.invoke_query(task.task_id, PREFIX + "state", {}))
            report["before"] = before
            if wave:
                target = copy.deepcopy(before["end_effector_poses"])
                target["gesture"] = "wave"
                target["gesture_arm"] = wave_arm
                report["observations"] = []
                report["wave"] = await execute(coordinator, task.task_id, target, report["observations"])
                after = outputs(await coordinator.invoke_query(task.task_id, PREFIX + "state", {}))
                report["after"] = after
                report["wave_arm"] = wave_arm
                wrist = [s[f"{wave_arm}_arm"]["joint_positions_rad"][6] for s in report["observations"]]
                report["wrist_range_rad"] = max(wrist) - min(wrist)
                report[f"peak_{wave_arm}_height_m"] = max(
                    s["end_effector_poses"][wave_arm]["position_m"][2]
                    for s in report["observations"])
                report[f"{wave_arm}_lift_m"] = (report[f"peak_{wave_arm}_height_m"]
                    - before["end_effector_poses"][wave_arm]["position_m"][2])
                report["return_error_m"] = {
                    side: math.dist(after["end_effector_poses"][side]["position_m"],
                                    before["end_effector_poses"][side]["position_m"])
                    for side in ("left", "right")
                }
                report["accepted"] = (report["wrist_range_rad"] >= .4
                    and report[f"peak_{wave_arm}_height_m"] >= 1.0) and all(
                    error <= .005 for error in report["return_error_m"].values())
                final = await coordinator.finalize_task(task.task_id)
                report["agent_task_status"] = final.status.value
                if not report["accepted"]:
                    raise RuntimeError("Measured greeting/return criteria not met")
                return
            if return_report:
                previous = json.loads(return_report.read_text())
                if previous.get("outbound", {}).get("data", {}).get("result", {}).get("status") != "succeeded":
                    raise RuntimeError("Return requires a recorded successful outbound Action")
                origin = previous["before"]["end_effector_poses"]
                report["source_report"] = str(return_report.resolve())
                report["return"] = await execute(coordinator, task.task_id, origin)
                after = outputs(await coordinator.invoke_query(task.task_id, PREFIX + "state", {}))
                report["after"] = after
                report["return_error_m"] = {
                    side: math.dist(after["end_effector_poses"][side]["position_m"], origin[side]["position_m"])
                    for side in ("left", "right")
                }
                report["accepted"] = all(value <= .005 for value in report["return_error_m"].values())
                final = await coordinator.finalize_task(task.task_id)
                report["agent_task_status"] = final.status.value
                if not report["accepted"]:
                    raise RuntimeError("Measured return criteria not met; inspect motion report")
                return
            origin = copy.deepcopy(before["end_effector_poses"])
            target = copy.deepcopy(origin)
            for pose in target.values():
                pose["position_m"][0] += .01
            report["outbound"] = await execute(coordinator, task.task_id, target)
            moved = outputs(await coordinator.invoke_query(task.task_id, PREFIX + "state", {}))
            report["moved"] = moved
            # A fresh return plan is admitted only after a known successful outbound Action.
            report["return"] = await execute(coordinator, task.task_id, origin)
            after = outputs(await coordinator.invoke_query(task.task_id, PREFIX + "state", {}))
            report["after"] = after
            displacement = {
                side: moved["end_effector_poses"][side]["position_m"][0] - origin[side]["position_m"][0]
                for side in ("left", "right")
            }
            return_error = {
                side: math.dist(after["end_effector_poses"][side]["position_m"], origin[side]["position_m"])
                for side in ("left", "right")
            }
            report.update(displacement_x_m=displacement, return_error_m=return_error)
            report["accepted"] = all(.003 <= value <= .02 for value in displacement.values()) and all(
                value <= .005 for value in return_error.values())
            final = await coordinator.finalize_task(task.task_id)
            report["agent_task_status"] = final.status.value
            if not report["accepted"]:
                raise RuntimeError("Measured movement/return criteria not met; inspect motion report")
        except BaseException as error:
            report["error"] = str(error) or type(error).__name__
            try:
                cancelled = await coordinator.cancel_task(task.task_id, reason=report["error"])
                report["cancellation_task_status"] = cancelled.status.value
            except Exception as cancel_error:
                report["cancellation_error"] = str(cancel_error)
            raise
        finally:
            report_path.write_text(json.dumps(report, indent=2) + "\n")
            print(f"AgentTask evidence: {report_path}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gateway", default="http://127.0.0.1:19083")
    parser.add_argument("--skills-root", type=Path, default=Path.home() / ".PhyAgentOS/skills")
    parser.add_argument("--workspace", type=Path, default=Path.home() / ".PhyAgentOS/g1d-validation")
    parser.add_argument("--execute", action="store_true",
        help="perform the supervised 1 cm bilateral out-and-back; default only reads state")
    parser.add_argument("--return-report", type=Path,
        help="return to poses from a report with a known successful outbound Action")
    parser.add_argument("--wave", action="store_true",
        help="one continuous greeting with observed wrist movement and return")
    parser.add_argument("--wave-arm", choices=("left", "right"), default="right",
        help="arm used for --wave (robot's own left/right)")
    asyncio.run(run(parser.parse_args()))


if __name__ == "__main__":
    main()
