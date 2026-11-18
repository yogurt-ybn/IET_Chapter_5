"""Compare each frozen policy in its training dynamics and in independently logged AirSim."""

import json, sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from split_routes import TEST, ROOT
from iet_ch5.trajectories import Trajectory, load_trajectory
from iet_ch5.learning.dataset import vehicle
from iet_ch5.allocation import Allocator
from iet_ch5.types import WrenchCommand
from controllers import policy_matrix
from wide_model import step
from wide_control import reference_grid, observation, DT


def compare(raw, out):
    report = {}
    with np.load(out / "nonlinear.npz") as data:
        model = dict(data)
    for route, source in [("agile_course", "agile_course"), (TEST, ROOT / (TEST + ".json"))]:
        report[route] = {}
        traj = Trajectory(load_trajectory(source))
        for seed in (731, 732, 733):
            folder = raw / f"{route}-ars_{seed}"
            params = vehicle(json.loads((folder / "metadata.json").read_text())["parameters"])
            allocator = Allocator(params)
            target, feed = reference_grid(traj.at, traj.duration, params)
            rows = [json.loads(l) for l in (folder / "log.jsonl").read_text().splitlines()]
            first = rows[0]
            s = first["truth"]
            x = np.r_[
                s["p"],
                s["v"],
                Rotation.from_matrix(s["R"]).as_euler("xyz"),
                s["omega"],
                np.asarray(first["telemetry_estimated_wrench"]) - model["hover"],
            ][None, :]
            matrix = policy_matrix(np.load(out / f"ars-{seed}.npz")["theta"])
            errors = []
            times = []
            for row in rows:
                t = row["t"]
                i = round(t / DT)
                if traj.tracking_interval[0] <= t <= traj.tracking_interval[1]:
                    errors.append(np.linalg.norm(x[0, :3] - row["reference"]["p"]) ** 2)
                    times.append(t)
                u = matrix @ observation(x[0], target[i]) + feed[i]
                achieved = allocator.allocate(WrenchCommand(u[0], u[1:])).predicted_wrench
                x = step(x, (achieved - model["hover"])[None, :], model)
            report[route][seed] = {
                "surrogate_rms_m": float(
                    np.sqrt(np.trapezoid(errors, times) / (times[-1] - times[0]))
                ),
                "airsim_rms_m": json.loads((folder / "result.json").read_text())["metrics"][
                    "loop_rms_m"
                ],
            }
    (out / "transfer.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    compare(Path(sys.argv[1]), Path(sys.argv[2]))
