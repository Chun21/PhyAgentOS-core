# Gateway operation deadline patch

The locked upstream Gateway commit 49caba94d641aff4664fbdf9d4c8b1402add1ce6
uses the HTTP handshake timeout as the Action lifecycle deadline.
`scripts/build_g1d_node.py` applies one checked source transformation:
for `g1d.dual_arm.execute_pose` only, lifecycle `deadline_ms` uses validated
`operation_deadline_s` (default 30, range 1–120). The request/acknowledgement
exchange retains its original 10-second maximum. No other Tool is affected.

The original source archive SHA256 stays locked. The built executable and
node archive hashes cover the transformed source. The companion patch shows
the replaced deadline expression; the build script contains the validation
and operation-specific selection in full. A missing/ambiguous source anchor
fails the build, so an upstream upgrade requires deliberate review.
