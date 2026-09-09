# UniRobot G1_D IK Model

Reference: Chun21 UniRobot checkout `7d99ebf5f13d0b2fd97ea8c295de356e526001c2`.
Algorithm sources: `src/robotree/kinematics/ik/pinocchio_casadi.py`,
`ik/dual_arm/g1_d_dex1.yaml`, and `src/robotree/utils/filters.py`.
These files and the model are used under Apache-2.0; see LICENSE.

The checkout's configured URDF is absent. Its running implementation loads
`src/robotree/kinematics/cache/g1_d_dex1_model_cache.pkl` instead. The SHA-256
of that exact model, joint order, bounds and graph hashes are in model.json.
The artifacts here were generated from that model using Pinocchio 3.1.0's
CasADi extension and CasADi 3.6.7. No pickle is loaded at deployment time.

The four serialized functions represent FK, translation error, SO(3) log
rotation error and zero-velocity/zero-acceleration RNEA gravity. They preserve
the reference model placements and inertias, including the Dex1 masses.
`L_ee` and `R_ee` are 5 cm along the wrist-yaw joint's local X axis.

`unirobot_g1d_fixed_base` is the **virtual fixed-base model frame** with the
reference base/lift/waist joints locked at zero. It is not a measured floor,
world, or actual mobile-base frame when those joints move. No live waist
angles are input to this IK. Old `g1d_base` targets must not be reinterpreted.

Rebuild with the trusted reference checkout and its conda toolchain:

```sh
python scripts/export_g1d_unirobot_ik.py --reference-root /path/to/unirobot \
  --output assets/robots/g1d/unirobot_ik
python scripts/export_g1d_ik_oracle.py --reference-root /path/to/unirobot \
  --output tests/fixtures/g1d_unirobot_ik.json
```

The runtime requires only CasADi 3.6.7 and NumPy for this solver; it neither
imports UniRobot nor requires a Pinocchio CasADi extension on the robot.
