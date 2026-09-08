"""Verify retained execution evidence without trusting a Tool success flag."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Sequence
from typing import Any, Literal

from PhyAgentOS.verification.contracts import CriterionVerdict, VerificationVerdict


def verify_execution(records: Sequence[dict[str, Any]]) -> VerificationVerdict:
    """Check integrity, identity, lifecycle and contiguous measured pose hold."""

    def verdict(
        status: Literal["success", "inconclusive"], reason: str, refs: list[str]
    ) -> VerificationVerdict:
        return VerificationVerdict(
            verdict=status,
            criteria=[
                CriterionVerdict(
                    criterion="g1d_robot_state_reached",
                    status="satisfied" if status == "success" else "unknown",
                    evidence_refs=refs,
                )
            ],
            reason=reason,
            lesson=reason,
        )

    refs: list[str] = []
    try:
        if [r["phase"] for r in records] != ["before", "during", "after"]:
            raise ValueError("before/during/after evidence is incomplete")
        for record in records:
            raw = json.dumps(
                {key: value for key, value in record.items() if key != "sha256"},
                sort_keys=True,
                allow_nan=False,
            ).encode()
            if hashlib.sha256(raw).hexdigest() != record["sha256"]:
                raise ValueError("evidence hash mismatch")
            refs.append("sha256:" + record["sha256"])
        for field in (
            "invocation_id",
            "attempt_id",
            "plan_id",
            "profile_digest",
            "runtime_instance_id",
        ):
            if len({r[field] for r in records}) != 1:
                raise ValueError("evidence identity mismatch")
        if records[2]["payload"]["status"] != "succeeded":
            raise ValueError("physical lifecycle did not succeed")
        frames = records[1]["payload"]["frames"]
        if not frames or any(
            b["monotonic_at"] <= a["monotonic_at"] or b["seq"] != a["seq"] + 1
            for a, b in zip(frames, frames[1:])
        ):
            raise ValueError("command evidence is incomplete or nonmonotonic")
        since = previous = None
        reached = False
        previous_frame = None
        plan = records[0]["payload"]["plan"]
        requested = {side: plan.get(f"dex1_{side}_opening") for side in ("left", "right")}
        for sample in records[1]["payload"]["observations"]:
            t = sample["monotonic_at"]
            frame = sample["state_frame"]
            if previous_frame is not None and not 0 < (frame - previous_frame) % (2**32) < 2**31:
                raise ValueError("replayed or regressing state evidence")
            previous_frame = frame
            if not math.isfinite(t):
                raise ValueError("nonfinite observation time")
            valid = (
                sample["safety_gate"] == "ready"
                and 0 <= sample["state_age_ms"] <= 100
                and all(math.isfinite(v) and 0 <= v <= 0.02 for v in sample["position_error_m"])
                and len(sample["position_error_m"]) == 2
                and all(math.isfinite(v) and 0 <= v <= 5 for v in sample["orientation_error_deg"])
                and len(sample["orientation_error_deg"]) == 2
            )
            for side, target in requested.items():
                if target is not None:
                    dex = sample["dex1"][side]
                    valid = (
                        valid
                        and dex["healthy"]
                        and dex["age_ms"] is not None
                        and 0 <= dex["age_ms"] <= 100
                        and dex["opening"] is not None
                        and abs(dex["opening"] - target) <= 0.05
                    )
            if not valid or (previous is not None and not 0 < t - previous <= 0.1):
                since = None
            if valid and since is None:
                since = t
            if since is not None and t - since >= 2:
                reached = True
            previous = t
        if not reached:
            raise ValueError("no continuous two-second bilateral acceptance window")
    except (KeyError, TypeError, ValueError, IndexError) as error:
        return verdict("inconclusive", str(error), refs)
    return verdict(
        "success", "Measured bilateral pose and requested openings met robot-state criteria", refs
    )
