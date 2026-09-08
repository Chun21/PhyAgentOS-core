# G1_D manipulation

Use this Skill to inspect Unitree G1_D arm-pair state and obtain bilateral pose plans.
This release has one `real-g1d` profile and is read-only. Its four Tools remain discoverable;
`execute_pose` reports `action_not_ready`, and `stop` reports `no_active_operation` with
`already_stopped` details. Reconcile these outcomes through the Gateway invocation ID.

Before an Action, inspect the relevant Tool context and bind the call to the current AgentTask.
Both arm target poses are required. Every pose supplies an explicit frame, positions in metres,
and a normalized `xyzw` quaternion. `plan_pose` is read-only and returns a short-lived plan;
its validity window is five seconds. Use `g1d_base`, which denotes the packaged model's
`pelvis` frame with the waist locked at neutral, and metric positions with normalized `xyzw`
quaternions. The Runtime subscribes to DDS state and uses its internal Pinocchio solver.
Startup, inspection, and planning publish no `rt/lowcmd` or control-mode requests.

Runtime readiness is not robot Action readiness. A healthy process or Gateway does not imply that
the safety gate, fresh state, approved mode, E-stop, or external Dex1 service are ready. Query
`g1d.dual_arm.state` before and after an Action, reconcile terminal state by invocation ID, and
never blindly retry an unknown outcome. `g1d.dual_arm.stop` is idempotent and only targets the
active invocation.

The existing Dex1 service is external to this Bundle. This slice reports its readiness as
absent and rejects plans requesting Dex1 opening. Physical execution, E-stop verification,
base-frame calibration, and supervised hardware acceptance remain separate delivery gates.

For installation and managed start/status/stop, read `READONLY.md` in this Bundle.
