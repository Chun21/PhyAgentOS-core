# G1_D deployment and physical acceptance

The current #17 Bundle delivers read-only startup, DDS state and internal
Pinocchio planning through the real Forge Gateway. Physical execution remains
unavailable. The execution and evidence components exercised by the older
conformance harness below are not yet a deployed execution service.

## 1. Build and install the read-only release

Use the `phyagent` conda environment on the build host:

```bash
conda activate phyagent
python scripts/package_g1d_release.py --dependency-cache /tmp/g1d-dependencies --output-dir dist/skills
sha256sum dist/skills/g1d-manipulation-0.2.0.tar.gz
```

The release builder verifies upstream sources and target dependency artifacts,
rebuilds the Node lock, and emits a self-verifying manifest-v2 archive. The
Bundle includes the real implementation, URDF, profile, licenses, locked
wheels, and fixed CycloneDDS sources. `scripts/package_skill.py` can still
package the source Bundle without the dependency cache.

For exact robot-host installation and conda setup, follow
[the shipped read-only workflow](../bundles/g1d-manipulation/READONLY.md).

## 2. Runtime operation and delivery status

Follow the same workflow for managed `start`, `status`, and `stop`, or a
foreground diagnostic launch. Discovery and Tool calls use the actual Forge
Gateway. `runtime_ready` never implies `action_ready`; execute and stop return
explicit pre-effect outcomes until the execution slice is delivered.

Automated acceptance includes a real installed Node, isolated loopback DDS,
real Pinocchio, independent URDF geometry checks and zero command publications.
Native aarch64 deployment and physical checks remain operator-confirmed gates.
The checklist and physical ladder below apply to later supervised execution;
they are not claims that this read-only release performs physical Actions.

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
