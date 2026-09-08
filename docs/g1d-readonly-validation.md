# G1_D read-only validation — issue #17

Validated on 2026-09-08 in the `phyagent` conda environment (CPython 3.12.14,
Pinocchio 4.1.0, NumPy 2.5.3, CycloneDDS Python 11.0.1, Dora 0.4.1).

| Check | Result |
| --- | --- |
| Complete repository pytest suite | 129 passed |
| Changed implementation/build/install modules, mypy | Passed, 7 files |
| Changed implementation/build/install/new tests, Ruff | Passed |
| Installed zipapp + official Forge Gateway + real loopback DDS | Passed |
| Foreground and Dora startup, queries, normal shutdown | Passed |
| Missing, stale, corrupt state; unknown profile; model/profile mismatches | Rejected explicitly |
| Bilateral asymmetric and near-limit real-model plans | Passed |
| Independent URDF chain, per-joint signs, TCP offset, limits | Passed |
| Unreachable target and invalid current-state/frame checks | Rejected explicitly |
| Lowcmd publications during installed-runtime test | Zero |
| aarch64 CPython wheels and conda toolchain resolution | Resolved; glibc override 2.31 |
| Native aarch64 install/build and physical robot validation | Not performed |

The test process uses loopback-only DDS in a non-robot domain. The installed
Node runs outside the checkout with the bundled implementation and model.
The host test substitutes host architecture in its local NodeInstaller lock
because the zipapp is architecture-neutral; the shipped aarch64 lock and
target dependency artifacts are kept intact. This is not native aarch64 evidence.

Reproduce the tests after installing the numerical/transport dependencies in conda:

```bash
conda activate phyagent
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -p pytest_asyncio.plugin -q
```

The release builder produced `dist/skills/g1d-manipulation-0.2.0.tar.gz`,
including all 43 locked target wheels and two locked CycloneDDS source archives.
SHA-256:
`5c04739955acd650de47785518db7d8284ca8a5d4ec7de3fafbc41eb2c4fc159`.
Both Bundle inventory and Node archive hashes are verified by the existing
installers. The conda environment is installed separately from its explicit lock.

## Standards review

Independent review: no documented-standard violations or actionable heuristic
smells. Runtime readiness and physical Action readiness remain distinct.

## Spec review

Independent review: supported-target native installation is still unverified.
The parent #9 jerk-limit and controlled-stop traceability remains in the
execution follow-up, not this read-only slice. No scope creep or additional
substantive defect was found. The subsequently exercised Dora stop timeout was
fixed by consuming its lifecycle events; both startup modes now pass.

Standards: 0 findings. Spec: 2 remaining validation/follow-up items; the primary
#17 gap is native aarch64 installation evidence. Keep #17 and parent #9 open
until their remaining acceptance gates are demonstrated.
