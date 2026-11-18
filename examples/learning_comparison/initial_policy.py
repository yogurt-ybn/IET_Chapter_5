"""Ablate reward training by deploying the exact initial policy on the same test tasks."""

import argparse
import json
from pathlib import Path
import numpy as np
from controllers import DirectPolicy, START
from excitation import flight

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for key in ("environment", "output"):
        p.add_argument(key, type=Path)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    path = args.output / "initial.npz"
    np.savez(path, theta=np.zeros(len(START)))
    for name in ("figure_eight", "agile_course"):
        res = flight(
            "initial_policy",
            DirectPolicy,
            {"policy": str(path)},
            name,
            args.output / name,
            json.loads(args.environment.read_text()),
        )
        print(name, res.status, res.metrics.get("loop_rms_m"), flush=True)
