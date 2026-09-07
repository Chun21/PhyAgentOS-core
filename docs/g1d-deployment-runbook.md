# G1_D Manipulation Skill — Deployment Runbook, Operator Checklist, and Acceptance Ladder

This document is the delivery surface for supervised, visual-free operation of the
`g1d-manipulation` Forge Skill on a Unitree G1_D (two 7-DoF arms, two Dex1-1 end
effectors). It covers robot-host installation, runtime operation, the mandatory
operator checklist, the robot-free conformance harness, and the staged physical
acceptance ladder. Nothing here connects to a robot on its own: every automated
step runs offline against fake DDS sources, fixture kinematics, and packaged
archives.

## 1. Robot-host installation (immutable Bundle/Node verification)

The deployment unit is the manifest-v2 Skill package
(`bundles/g1d-manipulation`, packaged by `scripts/package_skill.py` into
`g1d-manipulation-<version>.tar.gz`). Installation is explicit, hash-verified,
and atomic:

```bash
# On the development host: package and record the archive hash.
python scripts/package_skill.py bundles/g1d-manipulation /tmp/dist
sha256sum /tmp/dist/g1d-manipulation-0.1.0.tar.gz
```

```python
from pathlib import Path
from PhyAgentOS.skill_runtime.installer import SkillInstaller
from PhyAgentOS.skill_runtime.state import RuntimeStateStore

installer = SkillInstaller(root=Path("/opt/paos/skills"),
                           state_store=RuntimeStateStore(Path("/opt/paos/state")))
manifest = installer.install(Path("/tmp/dist/g1d-manipulation-0.1.0.tar.gz"),
                             expected_sha256="<recorded sha256>")
```

Rules enforced by the installer and the archive validator:

- The archive SHA-256 is verified before extraction; a tampered archive is
  rejected and the previously installed version is left untouched.
- Node artifacts are locked by `NodeLock` (id, version, platform, arch,
  SHA-256); the `g1d-runtime` executable is only accepted from its locked
  archive.
- Re-installation of a changed version backs up the previous install under
  `.backups/<skill>/<old-version>-<stamp>/`; a failed install restores it
  (rollback). `installer.remove("g1d-manipulation")` uninstalls cleanly, and
  refuses while the runtime is running.
- No credentials are stored in the Bundle, manifest, logs, or evidence.
  Host, DDS domain, URDF, and IK configuration are injected at start through
  the protected `real-g1d` profile's `required_environment`
  (`PAOS_G1D_DDS_DOMAIN`, `PAOS_G1D_URDF_PATH`, `PAOS_G1D_IK_CONFIG`) — the
  RuntimeManager preflight refuses to start without them. The Gateway binding
  is deliberately fixed to loopback by the dataflow so it can never be exposed
  publicly; reaching it is an explicit SSH-tunnel/robot-LAN decision made
  outside the Bundle.

## 2. Runtime operation (real-g1d start / status / stop)

The managed Robot-side Skill Runtime runs on the G1_D host (aarch64, Ubuntu
20.04, `eth0=192.168.123.164`). The development host reaches it over the robot
LAN or an SSH tunnel through the governed Tool API; the Gateway
(`127.0.0.1:19082` on the robot host) is never exposed publicly.

- **Start**: `RuntimeManager.start("g1d-manipulation", "real-g1d")` — or
  `MockSkillRuntime(BUNDLE).start("real-g1d")` for the offline rehearsal.
  Preflight verifies profile assets (`kinematics.json`, `dataflow.yaml`) and
  the locked `g1d-runtime` binary before launching the Dora flow.
- **Status / context**: `status()` distinguishes `runtime_ready` (healthy
  process group) from `action_ready` (safety gate, fresh CRC-valid state,
  approved mode, requested Dex1 health). `gateway_tools()` /
  `gateway_context(<tool_id>)` are the discovery surface; callers never guess
  Tool IDs or schemas.
- **Stop**: `stop()` is explicit and idempotent. Lifecycle is
  Idle → Arming → Active → Stop → Release; MotionSwitcher mode is never
  restored automatically, and an Action is never reconstructed or reposted
  after a crash or network loss — reconcile by persisted invocation ID.

Trusted Gateway access: only the explicit `real-g1d` profile exists in this
Bundle; the Gateway binds loopback on the robot host, agent mode is disabled,
tools mode enabled, and access crosses the boundary only via the robot LAN /
SSH tunnel from the trusted development host.

## 3. Operator run checklist (mandatory, human-confirmed)

These checks cannot be inferred from Tool availability; confirm each item
before and after every supervised run:

| # | Check | Before | After |
|---|-------|--------|-------|
| 1 | Network/DDS: robot LAN reachable, DDS domain matches the profile env | ☐ | ☐ |
| 2 | URDF/IK fixtures: `kinematics.json` present, slot layout verified (left 15–21, right 22–28) | ☐ | ☐ |
| 3 | MotionSwitcher: approved `mode_machine` active, ownership handed to the Runtime | ☐ | ☐ |
| 4 | Dex1-1 service: external service installed/running, left/right state fresh when an opening is requested | ☐ | ☐ |
| 5 | E-stop: physical E-STOP reachable and tested; software stop paths (operator stop, writer watchdog) exercised offline via the conformance harness | ☐ | ☐ |
| 6 | Obstacles: workspace clear for both arm envelopes | ☐ | ☐ |
| 7 | Load: declared object within the dual-arm payload envelope | ☐ | ☐ |
| 8 | Readiness: `g1d.dual_arm.state` shows fresh state, ready safety gate, `action_ready` true | ☐ | ☐ |
| 9 | Hashes: archive and Node SHA-256 match the recorded release values | ☐ | ☐ |

## 4. Conformance harness (robot-free)

`tests/test_g1d_conformance.py` is the acceptance harness. It drives the full
offline stack — bundle discovery, Tool schemas, planner, executor, Dex1
integration, evidence, installer — against fake DDS publishers, fixture
kinematics, and a controllable monotonic clock. Run it with:

```bash
env -u PYTHONPATH uv run --frozen python -m pytest tests/test_g1d_conformance.py -q
```

Scenario coverage (all without a robot):

- **Discovery/context**: Gateway tool discovery matches `tools.json` exactly;
  runtime readiness never masquerades as action readiness.
- **Query**: `plan_pose` validates and plans without side effects; `state`
  reports the safety gate and per-side Dex1 readiness.
- **Action lifecycle**: admission → hold → synchronized quintic streaming →
  terminal `succeeded` with CRC-valid frames.
- **Cancel/stop**: pre-effect stop cancels; operator stop during motion ends
  `stopped`; stop is idempotent.
- **Deadline/unknown**: the operation deadline ends `deadline_exceeded`; transport
  loss records `unknown` and never authorizes a blind retry.
- **Concurrency**: exactly one active Action; duplicate caller/plan retries
  reconcile to one invocation.
- **Binding**: a runtime restart rejects plans bound to the old runtime
  identity.
- **Evidence**: before/during/after artifacts link task, revision, binding,
  plan, and invocation identities; acceptance maps to PAOS verdicts; object
  acceptance stays inconclusive without an operator record.
- **Crash reconciliation**: a crashed runtime's invocation reconciles as
  unknown; a fresh runtime refuses to replay the old plan.
- **Rollback**: hash-verified install, tamper rejection leaving the install
  intact, versioned backups on upgrade, clean removal, offline archive
  validation.

## 5. Physical acceptance ladder (staged, supervised)

Each level must pass before the next; a failed level blocks progression.

1. **Offline fixtures** (automated): schema/fixture conformance via the
   harness above; no robot attached.
2. **Unloaded static dual-arm hold** (supervised): E-stop supervised, no
   load; verify the current-state hold stream, watchdog behavior, and stop
   paths on the real robot.
3. **Dex1 opening hold** (supervised): the external Dex1-1 service healthy;
   verify requested openings are commanded and gated on fresh state, without
   object interaction.
4. **Supervised object operation** (supervised, human confirmation required):
   a declared operation task under the operator checklist; object-level
   success requires the explicit confirmed operator record
   (`operator_confirmed_object_state` profile). Robot-state acceptance alone
   is not object acceptance.

## 6. What lives where

- Domain vocabulary: root `CONTEXT.md`.
- Tool contracts: `bundles/g1d-manipulation/tools/tools.json`.
- Offline seams: `PhyAgentOS/skill_runtime/g1d_{adapter,planner,executor,dex1,evidence}.py`.
- Packaging/install/rollback: `PhyAgentOS/skill_runtime/{archive,installer,manager}.py`.
- Harness: `tests/test_g1d_conformance.py` (plus per-seam test modules).
