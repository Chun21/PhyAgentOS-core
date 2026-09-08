# Robot-host diagnostic deployment

Date: 2026-09-08. Host: `192.168.123.164`, Ubuntu 20.04.6, aarch64.

The 0.3.0 release is extracted at
`/home/unitree/phyagent-deploy/0.3.0/bundle`. Its archive SHA256 is
`cc018a4b33aa3f2d97f2639432687fe18af26272262e735a060f63c7454a86e0`.
The independent conda environment is
`/home/unitree/miniconda3/envs/phyagent`.

This diagnostic installation uses 45 hash-verified packages from the conda
lock and the release's locked aarch64 wheels. The conda compiler/CMake packages
were omitted for the first diagnostic build because downloading them was slow.
CycloneDDS uses the host GCC 9.4.0 and CMake 3.16.3. This is a documented
toolchain deviation, not evidence of installing the complete locked toolchain.
Installation output is retained at
`/home/unitree/phyagent-deploy/0.3.0/install.log`.

Native installation completed: Python 3.12.14, Pinocchio 4.1.0, NumPy 2.5.3,
FastAPI 0.141.1, and CycloneDDS 11.0.1. The C library and Python extension
were built on aarch64, and `pip check` passed. The generated CycloneDDS wheel
SHA256 is `3114a15e8c8140d21e84cde2f2a4145af9e42c09dac9a6e2302b59406f28e133`.

The diagnostic Gateway is running at `http://192.168.123.164:19082` in
read-only mode, without automatic restart after reboot. Four tools are
discoverable at `/tools`; `POST /tools/g1d.dual_arm/state:invoke` with body
`{"arguments":{}}` returns `runtime_ready=true`, `action_ready=false`, and
`safety_gate=state_unavailable` at the time of this record. Logs are at
`/home/unitree/phyagent-deploy/0.3.0/runtime.log`.

The native loader needs `CYCLONEDDS_HOME=$CONDA_PREFIX/g1d-dds` and that
directory's `lib` on `LD_LIBRARY_PATH`. The first launch exposed this missing
environment setting; adding it allowed the unmodified 0.3.0 zipapp to start.

Read-only checks after the robot reboot:

- SSH is reachable and the reboot was confirmed through uptime.
- Factory MotionSwitcher `CheckMode` returned `(0, {"form": "0", "name": "ai"})`.
  No ReleaseMode or SelectMode call was made.
- DDS discovery found HG lowstate readers, including factory estimator and
  SLAM services, but no lowstate writer during the bounded scans.
- Both the independent PhyAgentOS reader and robot-side SDK reader received
  zero lowstate samples. Explicit wired-interface selection did not change it.
- External Dex1 left state was received through the factory SDK. No gripper
  commands were published.

Reproduce the PhyAgentOS state check from the repository on the development host:

```sh
conda activate phyagent
env -u PYTHONPATH python -m scripts.g1d_probe_state --seconds 5
```

The command constructs only a state reader, prints a bounded JSON summary, and
returns exit code 2 if no CRC-valid frame arrives. Discovery of a topic reader
does not establish the presence of a publisher or fresh feedback.

No robot movement, controller handoff, or acceptance of physical execution has
been performed. Other projects are not dependencies of this deployment.
