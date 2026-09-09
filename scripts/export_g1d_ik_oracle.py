"""Record numeric outputs from the installed, unmodified UniRobot IK."""

import argparse
import json
import sys
from pathlib import Path


def export(root: Path, output: Path):
    import numpy as np

    sys.path.insert(0, str(root / "src"))
    from robotree.kinematics.ik.unitree.g1 import G1_D_DEX1_ArmIK

    cases = []
    for current_initial in (True, False):
        solver = G1_D_DEX1_ArmIK()
        reference = np.asarray([.26, -.01, .01, 1.09, .006, .29, .009,
                                .24, -.008, .077, 1.26, .021, -.065, -.012])
        for step in range(8):
            target_q = reference.copy()
            target_q[[0, 7]] += .025 * min(step, 6)
            target_q[[3, 10]] -= .02 * min(step, 6)
            left, right = (pose.copy() for pose in solver.solve_fk_matrix(target_q))
            q, torque = solver.solve_ik(left, right, reference,
                                       use_current_q_as_initial=current_initial)
            cases.append({
                "reset": step == 0, "current_initial": current_initial,
                "reference": reference.tolist(), "left": left.tolist(), "right": right.tolist(),
                "q": q.tolist(), "torque": torque.tolist(),
                "fk": [pose.tolist() for pose in solver.solve_fk_matrix(q)],
            })
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(cases, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export(args.reference_root, args.output)
