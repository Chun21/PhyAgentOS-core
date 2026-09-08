# Installed read-only G1_D Runtime

This #17 slice starts the actual Forge Tool Gateway with an embedded
ToolEndpoint, the internal DDS bridge, and the real Pinocchio planner.
Exactly four Tools are exposed. `execute_pose` ends with `action_not_ready`;
`stop` ends with `no_active_operation` (`already_stopped` in error details).
These are pre-effect rejections; no operation is admitted to physical execution.

## Install on aarch64 Ubuntu 20.04

Prerequisites: conda, the installed PAOS control plane, and Dora CLI 0.4.1.

After installing the native CycloneDDS build, set these variables in the
activated conda environment before launching the Runtime or Dora:

```sh
export CYCLONEDDS_HOME="$CONDA_PREFIX/g1d-dds"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

The Python extension requires `libddsc.so.11` at runtime; `pip check` alone
does not validate the dynamic loader path. `scripts/g1d_readonly.sh` applies
these settings for a foreground diagnostic launch.
The dependency lock targets CPython 3.12 and glibc 2.31. The Node is a
hash-verified, self-extracting zipapp; it includes all Skill implementation,
the unchanged upstream Forge Gateway/ToolEndpoint source, configuration,
URDF and license notices. It needs no source checkout or implementation override.

1. Verify the release SHA-256 obtained from the release producer. Install the
   Bundle using `paos skill install /path/g1d-manipulation-0.3.0.tar.gz --local`.
2. Locate the installed Bundle (`paos skill inspect g1d-manipulation`), and
   set `G1D_BUNDLE` to that directory. Create the locked numerical/transport
   conda environment:

   ```bash
   conda create -n g1d-readonly --file "$G1D_BUNDLE/dependencies/conda-linux-aarch64.lock"
   conda activate g1d-readonly
   python "$G1D_BUNDLE/dependencies/install.py" --offline
   ```

   Release Bundles include all pinned Python wheels and CycloneDDS sources
   in `dependencies/cache`. The conda explicit lock downloads its pinned
   Python/build-toolchain packages from conda-forge. Source Bundles omit the
   cache; omit `--offline` to download and SHA-256-verify those artifacts first.
   CycloneDDS 11.0.1 has no upstream aarch64 Python wheel, so the installer
   builds its pinned C core and Python binding locally using the locked
   toolchain. The installed conda environment must be rebuilt if its prefix moves.
3. Install and verify the Node from the Bundle:

   ```bash
   paos forge-node install g1d-manipulation g1d-runtime --archive "$G1D_BUNDLE/artifacts/g1d-runtime-0.3.0.tar.gz"
   paos forge-node verify g1d-manipulation g1d-runtime
   ```

## Start, inspect, stop

Keep the activated conda environment's Python first on PATH when the PAOS
control plane launches Dora. Configure the approved robot DDS domain explicitly:

```bash
export PAOS_G1D_DDS_DOMAIN=0
paos skill start g1d-manipulation --profile real-g1d
paos skill status g1d-manipulation
curl --fail http://127.0.0.1:19082/tools
curl --fail http://127.0.0.1:19082/tools/g1d.dual_arm.state/context
curl --fail -H 'Content-Type: application/json' \
  -d '{"arguments":{}}' http://127.0.0.1:19082/tools/g1d.dual_arm/state:invoke
paos skill stop g1d-manipulation
```

For a foreground diagnostic start without Dora, run the installed
`g1d-runtime` executable with the same conda Python. It reads the model and
default profile from its own verified payload; `--help` lists host/port/profile
options. SIGTERM or Ctrl-C stops the Gateway. No Tool manages external services.

The Gateway binds loopback by default. Queries use the existing Forge
`data.response.result.outputs` envelope; Actions return an invocation ID and
their terminal rejection is read through `/invocations/{id}` and `/result`.
Context reports Runtime health separately from `action_ready=false` in
Gateway readiness and endpoint details. Missing, stale (>100 ms), CRC-invalid,
nonfinite, or faulted state cannot produce a plan. Replayed ticks do not refresh
state age. The source timestamp also prevents queued old DDS samples from
becoming fresh merely because they were read later.

Positions are metres and orientations normalized `xyzw`. `g1d_base` is the
retained URDF's **pelvis frame with neutral waist**, not a calibrated world or
floor frame. `L_ee`/`R_ee` are 5 cm along the local wrist-yaw X axis. The state
query exposes 7 ordered joint positions per arm in radians. Requested Dex1
opening is rejected until its readiness integration is delivered. Plans have
a five-second lifetime and are bound internally to Runtime identity, target,
ToolSpecs, frame/model configuration and trajectory profile.

## Evidence and remaining physical checks

The automated installed-runtime test uses the real Gateway, real CycloneDDS
on an isolated loopback domain, and the actual packaged numerical model.
It checks fresh state, asymmetric plans, strict inputs, CRC/stale failures,
Action rejection, lifecycle termination, and zero `rt/lowcmd` messages, both
in foreground mode and through an actual Dora 0.4.1 process lifecycle.
Independent URDF tree calculations check geometry, mounting offsets, signs
and FK; planner tests cover near-limit and unreachable targets.

These checks run on the development host with host-native numerical wheels.
Target aarch64 wheel availability, Python ABI and glibc compatibility are
checked separately by the locked dependency resolution. Native aarch64
installation/build and managed Dora startup still need robot-host confirmation.
No physical robot, E-stop status, controller ownership, firmware limits, or
world/base calibration has been verified by the automated checks. Before any
later execution release, perform the existing supervised acceptance ladder,
including read-only state/model calibration and small unloaded bilateral motion.

Model/source provenance and solver differences are in the Node payload's
`assets/robots/g1d/README.md`. The internal solver imports neither UniRobot /
robotree nor unitree_dsh. Upstream Forge Gateway is pinned to
`49caba94d641aff4664fbdf9d4c8b1402add1ce6`; its HTTP Tool routes, registry,
correlation, concurrency and invocation handling are retained unmodified.
The Skill uses an in-process Forge envelope carrier; Dora owns the node process.
