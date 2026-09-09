# Native Agent Control Validation

## Latest Hardware Run (2026-09-09)

The Agent -> Forge -> native DDS path completed continuous right-arm greetings.
The raised greeting ran from `/home/unitree/phyagent-deploy/raised-wave`, archive
SHA256 `c1a48a6e4cb3787a93233a0808474708beffb86e6982935cec36935f80ed2da5`.
Task `task_c2339fb09d06424a` and invocation
`gateway-46c61a492a3a4c1a8a954e9c01b032f6` completed with `succeeded` status.
Measured lift was 0.4184 m, wrist range 0.6825 rad, and right return error 0.00101 m.
The separate motion report remains `accepted=false`: peak model-frame height was
0.9891 m, below its 1.0 m criterion. Do not equate lifecycle success with acceptance.
The bounded session exited and confirmed factory `ai` restoration.

CRC processing now uses a verified native zlib equivalent; one shared state receiver
avoids duplicate DDS decoding. Retaining encoded per-frame evidence avoids the
observed approximately 40 ms Python cyclic-GC pause. Evidence serialization yields
between chunks and the HTTP endpoint streams persisted JSON. A hardware-free replay
on the robot host completed with approximately 3.7 ms maximum command spacing.
Real-session maximum writer gaps, including startup and terminal work, still reached
approximately 74 ms. These measurements do not establish hard real-time guarantees
or eliminate the need for operator supervision. Watchdog and state protections remain.

## Earlier Read-Only Validation (Historical)

Validated on 2026-09-08 in conda `phyagent`, using CycloneDDS core and Python binding
0.10.2. No other project's controller is used by this implementation.

Implemented native MotionSwitcher DDS RPC, supervised bounded control sessions with
independent hold/state monitoring, a 35-slot G1_D control profile, framework caller-ID
propagation, observed end-effector poses, and an AgentTask out-and-back validation CLI.
The IK model/solver remains the pinned unirobot implementation documented in
`g1d-unirobot-ik-alignment.md`.

Validation results:

- All 163 G1_D tests passed on the final rebuilt bundle.
- All 9 control-session tests passed, including real loopback DDS publication,
  discovery of the session's own writer, CRC-bearing hold frames, and recovery.
- The rebuilt installed zipapp passed all 3 launch-mode tests; the bound AgentTask
  simulation passed with both outward and return movement measured.
- Fixed a terminal-notification race: querying an available result before the
  completion event no longer closes the Gateway with EVENT_AFTER_TERMINAL.
- Ruff passed; mypy passed for gateway, read-only runtime and execution runtime.
- The Agent simulation uses a deterministic clock. It verifies protocol and lifecycle,
  not real-time scheduling or motor dynamics. Hardware scheduling remains unmeasured.

Robot deployment:

- Directory: `/home/unitree/phyagent-deploy/agent-control`.
- Archive SHA256: `db8117ed999c51ce5bb071acf0e364013d239864fdb1ae7aaae5c97e7f9980a9`.
- Read-only gateway: `http://192.168.123.164:19084`, PID 11185.
- Runtime instance: `g1d-7bbdd2a60c504f3e94c84268849722ec`.
- Agent Tool query returned fresh state (11.6 ms), mode_machine 5, both 7-joint
  observations and FK poses in `unirobot_g1d_fixed_base`.
- `action_ready=false` as expected for this read-only process.

Hardware acceptance is not complete. PID 149065 exited, but DDS discovery then found
another local `rt/lowcmd` writer, PID 160217, participant
`01102436-b1e5-777c-e925-0c22000001c1`. MotionSwitcher returned an empty active mode.
That writer was still running at the last check. No ReleaseMode, SelectMode or lowcmd
command has been sent to the robot by this work.

After the competing controller exits, verify discovery again, restore factory `ai`
only with no competing writer, and run the already-authorized supervised hold and
1 cm out-and-back. Record measured displacement, return error, writer gaps, and
confirmed `ai` restoration. See `bundles/g1d-manipulation/CONTROL.md` for commands.
The execution stream currently has zero gravity feedforward; matching IK alone does
not establish equivalence to unirobot's complete torque/controller behavior.
