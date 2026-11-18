"""Learn full discrete state increments using a physics-informed feature dictionary."""

import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from model import fit, read_flight
from split_routes import TRAIN, VALID
from iet_ch5.learning.dataset import vehicle


def features(x, u, hover):
    roll, pitch, yaw = x[:, 6:9].T
    sr, sp, sy = np.sin([roll, pitch, yaw])
    cr, cp, cy = np.cos([roll, pitch, yaw])
    axis = np.column_stack((cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr))
    p, q, r = x[:, 9:12].T
    qr = q * sr + r * cr
    # Kinematic features for Euler rates, projected rotor thrust, and gyroscopic products.
    return np.column_stack(
        (
            x[:, 3:],
            u,
            axis * (hover[0] + x[:, 12:13]),
            axis * (hover[0] + u[:, :1]),
            sp / np.maximum(cp, 0.2) * qr,
            (cr - 1) * q - sr * r,
            qr / np.maximum(cp, 0.2) - r,
            p * q,
            q * r,
            p * r,
            np.ones(len(x)),
        )
    )


def step(x, u, model):
    return x + features(x, u, model["hover"]) @ model["coef"]


def main(raw, output):
    old = Path(".local/identification")
    paths = [old / name for name in ("circle", "helix", "transfer")] + [
        raw / name for name in TRAIN
    ]
    parameters = json.loads((paths[0] / "metadata.json").read_text())["parameters"]
    (output / "vehicle.json").write_text(json.dumps(parameters, indent=2) + "\n")
    par = vehicle(parameters)
    hover = np.array([par.mass * par.gravity, 0.0, 0.0, 0.0])
    fit(paths, output / "linear.npz")
    designs, targets = [], []
    for path in paths:
        x, u = read_flight(path, hover)
        np.savez_compressed(output / f"fit-{path.name}.npz", state=x, action=u)
        designs.append(features(x[:-1], u[:-1], hover))
        targets.append(np.diff(x, axis=0))
    X = np.vstack(designs)
    Y = np.vstack(targets)
    scale = np.std(X, axis=0).clip(1e-7)
    scale[-1] = 1.0
    design = X / scale
    # Fixed ridge regularisation stabilises near-collinear thrust features.
    coef = (
        np.linalg.solve(design.T @ design + 1e-5 * np.eye(design.shape[1]), design.T @ Y)
        / scale[:, None]
    )
    np.savez(output / "nonlinear.npz", coef=coef, hover=hover)
    x, u = read_flight(raw / VALID, hover)
    report = {}
    for name, path in [
        ("old", "results/examples/learning_comparison/model.npz"),
        ("wide", output / "linear.npz"),
        ("nonlinear", output / "nonlinear.npz"),
    ]:
        with np.load(path) as data:
            model = dict(data)
        report[name] = {}
        for horizon in (1, 60, 72):
            ids = np.arange(0, len(x) - horizon, horizon)
            pred = x[ids].copy()
            for i in range(horizon):
                pred = (
                    step(pred, u[ids + i], model)
                    if "coef" in model
                    else pred @ model["A"].T + u[ids + i] @ model["B"].T + model["bias"]
                )
            delta = pred - x[ids + horizon]
            rel = Rotation.from_euler("xyz", pred[:, 6:9]).inv() * Rotation.from_euler(
                "xyz", x[ids + horizon, 6:9]
            )
            report[name][str(horizon)] = {
                "windows": len(ids),
                "position_m": float(np.sqrt(np.mean(np.sum(delta[:, :3] ** 2, axis=1)))),
                "velocity_m_s": float(np.sqrt(np.mean(np.sum(delta[:, 3:6] ** 2, axis=1)))),
                "attitude_deg": float(np.rad2deg(np.sqrt(np.mean(rel.magnitude() ** 2)))),
            }
    (output / "validation.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
