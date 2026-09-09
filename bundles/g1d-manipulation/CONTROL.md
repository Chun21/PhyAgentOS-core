# Supervised Arm Control

Use the `phyagent` conda environment with both CycloneDDS core and Python binding at
0.10.2. Native control has no dependency on another project's controller or Unitree SDK.
Stop other lowcmd controllers first. The operator must be present with an accessible
E-stop, unloaded arms, and clear workspace throughout the session.

The native session requires fresh CRC-validated state, mode_machine 5, factory `ai`
mode, exclusive local process lock, and DDS discovery without a competing writer.
It releases `ai`, waits for the command receiver, and holds the measured joint positions.
The initial profile bounds every arm joint to 0.20 rad from that initial position.
Only slots 12..28 are enabled: waist 12..14 holds its measured position; arms 15..28
follow validated plans. Other slots remain disabled. Gains match the pinned unirobot
G1_D configuration. Gravity feedforward is currently zero in the execution stream;
the aligned IK's gravity output is not applied by this controller.

On the robot, with the extracted bundle and repository launcher available:

```bash
conda activate phyagent
export PAOS_SKILL_ROOT=/home/unitree/phyagent-deploy/agent-control
export CYCLONEDDS_URI=file:///home/unitree/cyclonedds_ws/cyclonedds.xml
bash "$PAOS_SKILL_ROOT/g1d_control.sh" --operator-confirmed --session-seconds 120
```

The gateway listens on robot localhost port 19083. In another development-host terminal:

```bash
ssh -N -L 19083:127.0.0.1:19083 unitree@192.168.123.164
```

Use the AgentTask validation entry point from the repository:

```bash
conda activate phyagent
python scripts/g1d_agent_smoke.py --skills-root bundles
python scripts/g1d_agent_smoke.py --skills-root bundles --execute
```

For a raised right-arm greeting, start the supervised session with
`--max-arm-excursion-rad 1.5 --session-seconds 60`, then run
`python scripts/g1d_agent_smoke.py --skills-root bundles --wave --execute`.
The target is a bent-elbow pose at approximately `(0.415, -0.161, 1.052)` metres
in `unirobot_g1d_fixed_base`, with the end effector directed upward. These are model
coordinates, not height above the physical floor. The pinned IK solves the posture;
one continuous trajectory raises the right arm, waves its wrist, and returns to
the observed starting joints while holding the left arm. Acceptance requires a
measured model height of at least 1.0 m, wrist range of at least 0.4 rad, and return
position errors at most 5 mm. The override is capped at 1.5 rad; the default remains
0.20 rad, and joint, speed, state, ownership, and watchdog limits remain enforced.

Without `--execute`, the script only reads state. With it, Skill activation and live Tool
binding create a real AgentTask, then plan and execute 1 cm along fixed-base X for both
end effectors, and plan and execute return to the captured initial poses. Each Action
uses the framework's caller identity. The script stops on an unknown/failed Action;
it never assumes cancellation means physical stop. Its report requires at least 3 mm
measured displacement on each side and return position error no greater than 5 mm.
The AgentTask records execution facts with semantic verification disabled; the separate
motion report provides robot-state-only acceptance, not object-task acceptance.

The robot journal retains invocation/evidence records; `/g1d/evidence/{invocation_id}`
exposes them. Agent records and motion reports default to `~/.PhyAgentOS/g1d-validation`.
Unknown outcomes prevent further admission in that journal until reconciled.

Ctrl-C or session expiry ends the process and attempts confirmed restoration to `ai`.
State loss, DDS receiver/ownership loss, waist drift, excessive excursion, or stalled
trajectory updates also end the session. Check the final `g1d_control_session` record:
`recovery: ai_confirmed` confirms restoration. `unconfirmed` requires operator recovery.
A process kill or host failure cannot run this recovery path. DDS discovery is a
conflict detector, not a distributed ownership lock; do not start another controller
during the session. Software monitors do not replace the physical E-stop.

Real Agent-driven greetings have completed, including a measured 0.418 m lift.
The raised-greeting report remains unaccepted because its 0.989 m peak model height
fell below the 1.0 m criterion. See `docs/g1d-native-control-validation.md` in the
repository for measured results and remaining scheduling limitations. Simulated
AgentTask tests do not prove real motor movement or scheduling performance.
