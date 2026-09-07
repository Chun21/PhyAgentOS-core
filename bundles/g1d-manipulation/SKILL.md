# G1_D manipulation

Use this Skill for supervised, visual-free operation tasks on a Unitree G1_D with two 7-DoF
arms and optional Dex1-1 opening targets. The first release has one `real-g1d` profile and does
not include camera, simulation, locomotion, or whole-body tools.

Before an Action, inspect the relevant Tool context and bind the call to the current AgentTask.
Both arm target poses are required. Every pose supplies an explicit frame, positions in metres,
and a normalized `xyzw` quaternion. `plan_pose` is read-only and returns a short-lived plan;
`execute_pose` accepts only that plan identity. The Runtime owns SDK/DDS access and publishes
complete `rt/lowcmd` frames; callers never construct DDS messages.

Runtime readiness is not robot Action readiness. A healthy process or Gateway does not imply that
the safety gate, fresh state, approved mode, E-stop, or external Dex1 service are ready. Query
`g1d.dual_arm.state` before and after an Action, reconcile terminal state by invocation ID, and
never blindly retry an unknown outcome. `g1d.dual_arm.stop` is idempotent and only targets the
active invocation.

The existing Dex1 service is external to this Bundle. Normal Tool calls do not install, restart,
calibrate, or stop it. Object-level success requires an explicit operator confirmation in the
no-camera stage; robot-state acceptance alone is not object acceptance.
