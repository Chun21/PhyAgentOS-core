# G1_D manipulation

Use this Skill to inspect Unitree G1_D arm-pair state and plan and execute bilateral poses.
The `real-g1d` profile starts read-only by default. An operator may explicitly start a
control session using `paos agent --physical` (lifetime follows the TUI; see `CONTROL.md`). Require `action_ready` before execution;
the four Tools remain discoverable when execution is unavailable.

For a user request to wave, use one continuous `plan_pose` call with the current
observed bilateral poses, set `gesture` to `wave`, select `gesture_arm: "left"`
for 左臂/左手 or `gesture_arm: "right"` for 右臂/右手, and then submit its returned
`plan_id` to `execute_pose`. Do not invent joint values or call the low-level DDS
tools directly. If no arm is requested, default to right. The Runtime supplies the
selected arm's greeting target and solves IK; the Agent does not invent greeting
coordinates. One continuous trajectory raises that arm, oscillates its wrist,
and returns to the observed starting joints. The other arm holds its measured
joints throughout. Left/right refer to the robot's own sides, not the observer's.

Other supported motions use `plan_pose` without `gesture`: for example move the
left hand up 5 cm by adding 0.05 to its observed position Z while keeping the right
pose unchanged. Both arm pose targets are required. Arbitrary tasks still need
reachable targets, valid plans and readiness; do not claim every requested gesture
is a built-in preset. Simultaneous two-arm waving is not currently a preset.

Before an Action, inspect the relevant Tool context and bind the call to the current AgentTask.
Both arm target poses are required. Every pose supplies an explicit frame, positions in metres,
and a normalized `xyzw` quaternion. `plan_pose` is read-only and returns a short-lived plan;
its validity window is five seconds. Use `unirobot_g1d_fixed_base`, the UniRobot
virtual model frame with base/lift/waist locked at zero. This is not the physical
floor/world frame or a measured mobile-base frame. Real waist angles may be nonzero;
they are outside the arm-only IK. Reject old `g1d_base` targets instead of relabeling
their coordinates. The Runtime uses the pinned UniRobot CasADi/IPOPT algorithm.
Default startup, inspection, and planning publish no `rt/lowcmd` or control-mode requests.
An explicitly enabled control session takes over and continuously holds the current pose
between Actions. Waist position is held, never included as an IK target.

The state Tool returns `end_effector_poses.left` and `.right` in the same frame used for
planning. Use these observed poses for relative targets. Submit only `plan_id` and the
optional operation deadline to `execute_pose`: the Agent supplies the durable caller ID.
Wait for a known successful terminal result before planning the next operation. On
unknown outcome, stop the sequence and reconcile; never automatically retry or return.

Runtime readiness is not robot Action readiness. A healthy process or Gateway does not imply that
the safety gate, fresh state, approved mode, E-stop, or external Dex1 service are ready. Query
`g1d.dual_arm.state` before and after an Action, reconcile terminal state by invocation ID, and
never blindly retry an unknown outcome. `g1d.dual_arm.stop` is idempotent and only targets the
active invocation.

The existing Dex1 service is external to this Bundle. Native arm-only control reports it
as absent and rejects plans requesting Dex1 opening. E-stop and physical clearance require
the operator; DDS state cannot verify them. Hardware acceptance must use measured movement,
not merely a successful Tool response.

For installation and managed start/status/stop, read `READONLY.md` in this Bundle.
