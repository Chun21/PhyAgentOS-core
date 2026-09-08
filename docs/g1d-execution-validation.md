# G1_D execution delivery (#18–#20)

## Implemented software

Bundle 0.3.0 retains exactly four Tools and defaults to read-only startup.
Execution is enabled only with all three arguments:

```sh
conda run -n phyagent python /path/to/g1d-runtime \
  --control-profile /etc/phyagent/g1d-control.json \
  --authority-socket /run/robot-supervisor/g1d.sock \
  --journal /var/lib/phyagent/g1d-actions.sqlite
```

The control JSON contains `approved_mode`, complete 35-element `modes` and
`gains` arrays (each gain entry is `[kp,kd]`), and optional `dex1` containing
`verified_open_q: 5.4` and `verified_closed_q: 0.0`. Values must come from
the approved hardware configuration. Test gains are not a deployment profile.
Arm slots 15–28 require enabled position control and positive gains. Every
non-arm slot retains its initial position and verified configured mode/gains.

The supervisor is an external deployment dependency. This repository supplies
the lease client, not a substitute supervisor that asserts authority without
controlling the other robot services. A journal lock prevents duplicate
processes using that journal; it does not establish global lowcmd ownership.

## Supervisor contract

The operator-managed Unix socket accepts one newline-terminated JSON request:

```json
{"operation":"acquire_or_renew","owner_id":"g1d-controller-...","profile_digest":"...","lease_ms":50}
```

It returns the same identity/digest/lease plus boolean `exclusive`,
`handoff_approved`, `operator_ready`, and `constraints_verified`. All must be
true. The supervisor must exclusively reserve lowcmd across every competing
controller, verify the requested profile and manual preflight, and revoke the
lease on mode/ownership/safety loss. Secure socket permissions are required.
Lease expiry is measured conservatively from request start. Renewal runs on
a separate thread; the writer never blocks on a supervisor socket exchange.
Neither startup nor shutdown automatically calls MotionSwitcher or restores
another controller. Lease expiry is not evidence of a physical stop.

## Requirement traceability

| Requirement | Implementation / verification |
| --- | --- |
| Current plans, bilateral start binding | planner and executor; changed/missing start rejected, five-second TTL |
| Durable caller/plan identity | SQLite FULL synchronous admission before writes; process lock; terminal retention; restart UNKNOWN |
| 500 Hz | dedicated `G1DControlLoop`, monotonic timestamps, no replay bursts; watchdog test and actual scheduler test |
| Quintic v/a/jerk | shared analytic duration, per-joint bounds; polynomial bounded deceleration with extrema checks |
| Stop and faults | fresh observed hold; stale/mode/safety/write/scheduler failures UNKNOWN; unresolved outcome blocks new execution |
| Physical acceptance | actual Pinocchio FK, 2 cm/5 degrees for two seconds; requested Dex1 health/opening |
| Forge lifecycle | upstream handler, Gateway identities, query/status/result/cancel/stop; request and operation deadlines separated by pinned patch |
| Evidence | SQLite before/during/after payloads and hashes; `/g1d/evidence/{invocation_id}`; PAOS `verify_execution` |
| External Dex1 | GO MotorCmds/MotorStates DDS, loopback test; normalized 0 closed / 1 open; absent optional sides do not gate arms |

`tests/test_g1d_executor.py` covers controlled clock/feedback faults and
trajectory behavior. `tests/test_g1d_readonly_runtime.py` tests production
composition with actual FK. `tests/test_g1d_action_endpoint.py` tests the
packaged Forge protocol library and production Gateway composition.
`tests/test_g1d_installed_gateway.py` installs and runs the zipapp directly and
through Dora with real loopback DDS, retaining zero-command default checks.
Its execution case uses the installed zipapp, a test-only supervisor socket,
real loopback HG DDS and the production model/scheduler/Gateway/evidence path.
`tests/test_g1d_dex1_bridge.py` exercises actual loopback DDS GO messages.

The in-process Gateway execution test permits the truthful UNKNOWN result
when the development host misses the watchdog bound. It is not native timing
acceptance. Deterministic feedback tests require observed SUCCEEDED.

## Outstanding acceptance and operational limits

- Full installed execution fault matrix over controlled DDS remains to be run;
  installed execution coverage does not yet reproduce every deterministic
  executor fault test through HTTP.
- The real robot's supervisor implementation, approved gains/modes/jerk limits,
  controller handoff and deployed Dex1 calibration require site evidence.
- Native aarch64 conda installation/build and measured p99 writer jitter,
  monitor latency, state age, unloaded hold and small bilateral movements
  remain #19 hardware acceptance. No physical commands were sent here.
- Dex1 opening holds and object conditions require supervised #20 runs and
  explicit human confirmation. Robot-state verification does not infer object
  placement, grasp stability or task-level object success.
- Crash reconciliation retains admission and before evidence. Buffered during
  observations may be missing after a crash; UNKNOWN and incomplete evidence
  remain explicit. Do not delete the journal to retry an unresolved operation.
- Shutdown/transport failure cannot confirm a physical stop. The operator must
  use the external safety/recovery procedure. No automatic ownership restore.
- Evidence finalization is synchronous after execution; native timing and
  query-latency acceptance must include this cost.

Use `docs/g1d-deployment-runbook.md` for the physical checklist. Issues #18–#20
must not be closed solely on these offline tests.
