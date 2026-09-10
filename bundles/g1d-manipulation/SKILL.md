# G1_D manipulation

Use this Skill to see through Unitree G1_D cameras, operate left/right Dex1 grippers,
and plan and execute bilateral arm poses. Use it for 看相机、观察桌面、张开夹爪、
闭合左/右夹爪 and manipulation requests as well as arm gestures.
The `real-g1d` profile starts read-only by default. An operator may explicitly start a
control session using `paos agent --physical` (lifetime follows the TUI; see `CONTROL.md`). Require `action_ready` before execution;
the seven Tools remain discoverable when execution is unavailable.

For visual requests, call `g1d.dual_arm.camera_state`, then
`g1d.dual_arm.camera_observe` with `camera: "head"`, `"left_wrist"` or
`"right_wrist"` and an optional `question`. The native PAOS query wrapper sends
the actual JPEG to the configured vision model and returns `outputs.vision`.
Only `vision.status: succeeded` means the model has analyzed the image. If vision
fails, report the error rather than describing unseen objects. Camera queries
need no physical session and never publish motion commands. The head image is
side-by-side stereo; wrist cameras are monocular. Camera names refer to robot sides.
Each query requires a newly received frame. Receipts are timestamped locally;
the camera transport does not supply exposure time. Images/vision descriptions
are historical observations, not continuous feedback. Obtain a new observation
after a movement or scene change. Images expire at the Gateway after 60 seconds.

For gripper-only requests use `g1d.dual_arm.plan_gripper`, passing `left` and/or
`right` opening in [0, 1] (0 closed, 1 open), then `execute_pose` with the returned
plan ID. An omitted gripper stays unchanged. This holds both measured arm joint
positions exactly and does not solve IK. Check the requested side's `dex1` state
first; an absent other side does not block it. Commands ramp at at most 0.5 opening/s
and refresh between Actions for the physical TUI lifetime. `state.gripper_control`
reports active targets and any refresh fault. A stop/failure freezes
the last commanded opening; lost feedback/ownership stops that side's refresh.
Exiting the physical session stops refresh and the external service enters BRAKE
after its 1 s timeout. Do not promise that an object remains held after TUI exit.

For combined arm and finger poses, `plan_pose` accepts optional `dex1.opening`
inside each target. Prefer sequential approach, gripper closure, and visual
verification for a grasp; do not close while approaching unless explicitly intended.
Successful gripper execution verifies opening within 0.05 for two seconds. It
does not establish object contact, grip force or successful grasp. If an object
prevents reaching the requested opening, the Action may time out; reconcile the
measured opening and a fresh image, and do not automatically squeeze harder.

Current cameras are RGB without verified depth, intrinsics or robot extrinsics;
`spatial_targeting_ready` is false. Do not turn estimated image pixels into
`unirobot_g1d_fixed_base` metres or claim autonomous visual grasping. You may
identify objects, inspect whether a gripper appears open/closed, and execute
user-supplied Cartesian goals or explicit relative moves through the normal
planner. An object-only command such as “抓住桌上的杯子” needs a calibrated spatial
target before approach/lift. Explain that specific missing capability instead of
claiming that cameras or grippers are unavailable.

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

The existing Dex1 service is external to this Bundle and is not installed, calibrated
or restarted by an Agent Tool. Native physical control subscribes to its left/right
state and enables commands only when requested; arm-only actions leave fingers unchanged.
E-stop and physical clearance require
the operator; DDS state cannot verify them. Hardware acceptance must use measured movement,
not merely a successful Tool response.

For installation and managed start/status/stop, read `READONLY.md` in this Bundle.
