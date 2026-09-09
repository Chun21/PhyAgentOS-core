# UniRobot IK Alignment

Reference: UniRobot commit `7d99ebf5f13d0b2fd97ea8c295de356e526001c2`,
`G1_D_DEX1_ArmIK`, with the operator's
`dex1_internal_pico_xrobotoolkit.yaml` (`use_current_q_as_initial: true`).

Production Runtime now uses `g1d_kinematics_unirobot.UniRobotKinematics`.
The previous DLS implementation remains available for historical model tests,
but is not the production Runtime solver. The exported model comes from the
exact cache used by the reference checkout; its configured URDF was absent.
Provenance, licensing and export instructions are in
`assets/robots/g1d/unirobot_ik/README.md`.

Aligned behavior:

- Fourteen arm joints, identical placements, limits, end-effector offsets and inertias.
- CasADi 3.6.7/IPOPT, translation/rotation/regularization/smoothing weights
  `50 / 0.35 / 0.02 / 0.1`, and the reference solver options.
- Current measured arm state as initial guess and smoothness reference.
- Persistent float32 weighted filter `[0.4, 0.3, 0.2, 0.1]`, including duplicate
  suppression and the unfiltered first three samples.
- RNEA gravity torque at the filtered solution. This is returned by the solver;
  the existing trajectory executor is not changed to send feedforward torque.
- Reference API failure fallback returns current q and zero torque. The Agent
  planning adapter explicitly rejects that outcome instead of reporting success.

The Tool frame is now `unirobot_g1d_fixed_base`, the virtual reference model
frame. Old `g1d_base` targets are rejected, not silently reinterpreted. Live
waist angles are outside arm IK, so nonzero waist no longer blocks planning.
No additional waist movement is introduced. This does not establish physical
world/base calibration, collision avoidance, or control ownership.

Validation on 2026-09-08:

- 152 G1_D tests passed, with two existing Starlette deprecation warnings.
- Ruff passed for changed production/export modules; mypy passed three runtime files.
- Sixteen sequential reference cases cover both current-state and previous-solution
  initial guesses, filter warmup/history, angles, gravity torque and FK.
- The same oracle ran from the packaged zipapp on the robot's aarch64 `phyagent`
  conda environment: maximum angle error 0 rad, torque error 0 Nm, FK matrix
  difference `6.94e-18` for those cases. This is numerical evidence for those
  inputs, not a universal bitwise-equivalence claim.
- The robot Gateway accepted a current-pose bilateral plan from actual DDS
  state (mode_machine 5, state age about 6 ms). No command was published.

Diagnostic deployment: `/home/unitree/phyagent-deploy/ik-aligned`, Gateway
`http://192.168.123.164:19082`. The new source bundle SHA-256 is
`8dd0edbda14854ec79d96930d217b9f284e4d0236023e22fb65f6def91be2dea`.
It uses the existing conda dependencies plus the locked CasADi aarch64 wheel;
this small diagnostic bundle omits the offline dependency cache. The prior
0.3.0 deployment remains available. No release was published.

IK alignment does not complete physical execution: supervisor handoff,
approved gains/holding policy and supervised movement acceptance remain open.
