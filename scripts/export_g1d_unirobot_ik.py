"""Export the trusted UniRobot G1_D cache using its original conda toolchain.

Build-time reference only. The deployed Skill never imports UniRobot or pickle.
"""

import argparse
import hashlib
import json
import pickle
from pathlib import Path


def export(root: Path, output: Path) -> None:
    import casadi
    import pinocchio as pin
    from pinocchio import casadi as cpin

    cache = root / "src/robotree/kinematics/cache/g1_d_dex1_model_cache.pkl"
    digest = hashlib.sha256(cache.read_bytes()).hexdigest()
    if digest != "cb9553802a29b1d15c9d4d8d6e0b9500701fd5337ef14839c1f3de49f786749a":
        raise ValueError("reference cache changed; review before exporting")
    if pin.__version__ != "3.1.0" or casadi.__version__ != "3.6.7":
        raise ValueError("export requires reference Pinocchio 3.1.0 and CasADi 3.6.7")
    model = pickle.loads(cache.read_bytes())["reduced_model"]
    symbolic = cpin.Model(model)
    data = symbolic.createData()
    q = casadi.SX.sym("q", model.nq)
    left = casadi.SX.sym("left", 4, 4)
    right = casadi.SX.sym("right", 4, 4)
    cpin.framesForwardKinematics(symbolic, data, q)
    poses = [data.oMf[model.getFrameId(name)] for name in ("L_ee", "R_ee")]
    functions = {
        "fk": casadi.Function("fk", [q], [pose.homogeneous for pose in poses]),
        "translation": casadi.Function("translational_error", [q, left, right], [
            casadi.vertcat(*(pose.translation - target[:3, 3]
                            for pose, target in zip(poses, (left, right))))]),
        "rotation": casadi.Function("rotational_error", [q, left, right], [
            casadi.vertcat(*(cpin.log3(pose.rotation @ target[:3, :3].T)
                            for pose, target in zip(poses, (left, right))))]),
        "gravity": casadi.Function("gravity", [q], [cpin.rnea(
            symbolic, symbolic.createData(), q,
            casadi.SX.zeros(model.nv), casadi.SX.zeros(model.nv))]),
    }
    output.mkdir(parents=True, exist_ok=True)
    hashes = {}
    for name, function in functions.items():
        path = output / f"{name}.casadi"
        function.save(str(path))
        hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    metadata = {
        "reference_commit": "7d99ebf5f13d0b2fd97ea8c295de356e526001c2",
        "reference_cache_sha256": digest,
        "pinocchio": pin.__version__, "casadi": casadi.__version__,
        "joint_names": list(model.names)[1:],
        "lower": model.lowerPositionLimit.tolist(),
        "upper": model.upperPositionLimit.tolist(),
        "frame": "unirobot_g1d_fixed_base",
        "functions": hashes,
    }
    (output / "model.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export(args.reference_root, args.output)
