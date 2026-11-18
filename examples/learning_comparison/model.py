"""Local, controlled state-transition identification from full logged flight transitions."""

import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def read_flight(path, hover):
    result = json.loads((path / "result.json").read_text())
    if result["status"] != "passed":
        raise ValueError(f"Incomplete acquisition: {path}")
    rows = [json.loads(line) for line in (path / "log.jsonl").read_text().splitlines()]
    states = [r["truth"] for r in rows]
    angles = Rotation.from_matrix([s["R"] for s in states]).as_euler("xyz")
    state = np.column_stack(
        (
            [s["p"] for s in states],
            [s["v"] for s in states],
            angles,
            [s["omega"] for s in states],
            np.array([r["telemetry_estimated_wrench"] for r in rows]) - hover,
        )
    )
    action = np.array([r["steady_predicted_wrench"] for r in rows]) - hover
    if np.max(np.abs(np.diff([r["t"] for r in rows]) - 0.012)) > 0.0001:
        raise ValueError("Unexpected control interval")
    return state, action


def fit(paths, destination):
    params = json.loads((paths[0] / "metadata.json").read_text())["parameters"]
    hover = np.array([params["mass"] * params["gravity"], 0.0, 0.0, 0.0])
    designs, targets = [], []
    for path in paths:
        x, u = read_flight(path, hover)
        # Learn increments of every state. Absolute position is translation invariant.
        designs.append(np.column_stack((x[:-1, 3:], u[:-1], np.ones(len(x) - 1))))
        targets.append(np.diff(x, axis=0))
    design = np.vstack(designs)
    scale = np.maximum(np.std(design, axis=0), 1e-7)
    scale[-1] = 1.0
    coef, _, rank, _ = np.linalg.lstsq(design / scale, np.vstack(targets), rcond=1e-8)
    coef = (coef / scale[:, None]).T
    if rank != design.shape[1]:
        raise ValueError("Insufficient excitation for local model")
    A = np.eye(16)
    A[:, 3:] += coef[:, :13]
    np.savez(destination, A=A, B=coef[:, 13:17], bias=coef[:, -1], hover=hover)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("collection", type=Path)
    p.add_argument("model", type=Path)
    args = p.parse_args()
    fit([args.collection / s for s in ("circle", "helix", "transfer")], args.model)
