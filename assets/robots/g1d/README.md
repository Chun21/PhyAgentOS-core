# G1_D robot description

Unitree arm-pair URDF (waist, two 7-DoF arms and finger joints).
Used by the g1d-manipulation Skill for kinematics-only model
building (`pin.buildModelFromUrdf`, no geometry/meshes required).

Source: Unitree robot description, vendored from the local UniRobot checkout
(`assets/g1_d/g1_d.urdf`); meshes are intentionally not vendored.

The exact retained model SHA-256 is
`0e06766885ae6342616cd636dffe858f3f063d14e526557edb6ff818342a0ed9`.
The file is unmodified from the accepted `07a68c5` baseline. Its original
upstream commit was not recorded. The reference checkout inspected for #17 is
`https://git.unitree.com/unitree/rd/manipulation/unirobot.git` at
`7d99ebf5f13d0b2fd97ea8c295de356e526001c2`; its Apache-2.0 license is retained
in [LICENSE](LICENSE). This is source/model attribution, not a runtime dependency.

`g1d_base` denotes this model's root `pelvis`, not the floor, wheeled chassis,
or world frame. Waist joints and finger joints are locked at neutral; the
14 retained joints map left to DDS slots 15–21 and right to 22–28, with radians
and no sign inversions. `L_ee` and `R_ee` add `[0.05, 0, 0]` metres to their
wrist-yaw joint frames, matching the reference Dex1 mounting configuration.
Planning rejects non-neutral waist slots 12–14. Actual waist/base mounting and
zero calibration must still be checked against the deployed G1_D.

Current UniRobot `g1_d_dex1.yaml` uses a different full model with wheel/lift
joints. That model's world/base placement is not interchangeable with the
retained `pelvis` frame. The offline tests independently traverse the packaged
URDF and verify the retained geometry; they do not establish hardware calibration.

The profile's 14 asymmetric position-limit pairs are checked against this
URDF and the reduced Pinocchio model on every startup. They replace the former
uniform ±2.8 rad demonstration limit. They are model limits, not measured
firmware limits. Velocity/acceleration bounds remain conservative planning policy.

Reference algorithm: `src/robotree/kinematics/ik/pinocchio_casadi.py` uses
CasADi/IPOPT, weighted position/rotation/posture/smoothness costs and a moving
filter. The independent internal solver uses local SE(3) error, damped least
squares, a projected posture preference, and joint-limit clipping. It has no
CasADi, robotree, Unitree SDK, or unitree_dsh import. Every accepted proposal
receives bilateral FK error and limit checks; failure to converge is explicit.
