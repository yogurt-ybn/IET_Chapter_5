"""Recompute traversal errors from AirSim state samples and summarise the frozen runs."""

import hashlib
import json
from pathlib import Path
import sys
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from model import read_flight
from split_routes import TEST, VALID, TRAIN


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def envelope(x):
    orientation = Rotation.from_euler("xyz", x[:, 6:9]).as_matrix()
    return np.column_stack(
        (
            np.linalg.norm(x[:, 3:6], axis=1),
            np.degrees(np.arccos(orientation[:, 2, 2].clip(-1, 1))),
            np.abs(np.degrees(x[:, 8])),
            np.linalg.norm(x[:, 9:12], axis=1),
        )
    )


def main(raw, out):
    hover = np.load(out / "nonlinear.npz")["hover"]
    x, u = read_flight(raw / "collection" / VALID, hover)
    np.savez_compressed(out / "validation_flight.npz", state=x, action=u)
    old = ["circle", "helix", "transfer"]
    summary = {}
    for names, label in [(old, "original"), ([*old, *TRAIN], "expanded")]:
        values = np.vstack([envelope(np.load(out / f"fit-{name}.npz")["state"]) for name in names])
        summary[label] = {
            "samples": len(values),
            "max_speed_tilt_yaw_bodyrate": values.max(axis=0).tolist(),
        }
    (out / "coverage.json").write_text(json.dumps(summary, indent=2) + "\n")
    sources = {}
    for path in [
        *(Path(".local/identification") / name for name in old),
        *(raw / "collection" / name for name in (*TRAIN, VALID)),
    ]:
        sources[path.name] = {
            name: digest(path / name) for name in ["log.jsonl", "physics.jsonl", "metadata.json"]
        }
    (out / "acquisition_hashes.json").write_text(json.dumps(sources, indent=2) + "\n")
    for file, value in json.loads((out / "frozen.json").read_text()).items():
        assert digest(out / file) == value
    rows = []
    traces = {}
    for route in ["agile_course", TEST]:
        settings = []
        for path in sorted((raw / "evaluation").glob(route + "-*")):
            name = path.name[len(route) + 1 :]
            meta = json.loads((path / "metadata.json").read_text())
            result = json.loads((path / "result.json").read_text())
            log = [json.loads(line) for line in (path / "log.jsonl").read_text().splitlines()]
            settings.append([meta["parameters"], meta["config"]["task_config"], meta["feedback"]])
            t = np.array([r["t"] for r in log])
            truth = np.array([r["truth"]["p"] for r in log])
            ref = np.array([r["reference"]["p"] for r in log])
            err = np.linalg.norm(truth - ref, axis=1)
            config = meta["config"]["task_config"]
            start = config["transfer_onset"]
            end = start + config["transfer_duration"]
            mask = (t >= start) & (t <= end)
            rms = float(np.sqrt(np.trapezoid(err[mask] ** 2, t[mask]) / (t[mask][-1] - t[mask][0])))
            assert abs(rms - result["metrics"]["loop_rms_m"]) < 1e-7
            rows.append(
                {
                    "route": route,
                    "method": name,
                    "status": result["status"],
                    "failure": result["failure"],
                    "rms_m": rms if t[-1] >= end else None,
                    "observed_interval_rms_m": rms,
                    "last_time_s": float(t[-1]),
                    "final_tilt_rad": log[-1]["constraints"]["tilt_rad"],
                    "full_traversal": bool(t[-1] >= end),
                    "peak_m": float(err[mask].max()),
                    "saturation_fraction": result["metrics"]["saturation_fraction"],
                    "collisions": result["metrics"]["collision_samples"],
                    "log_sha256": digest(path / "log.jsonl"),
                }
            )
            traces[route, name] = (t, truth, ref, err)
            np.savez_compressed(
                out / (path.name + ".npz"),
                t=t,
                truth_p=truth,
                reference_p=ref,
                motor_pwm=[r["motor_pwm"] for r in log],
                requested_wrench=[r["requested_wrench"] for r in log],
            )
        assert len(settings) == 10 and all(s == settings[0] for s in settings)
    (out / "tracking.json").write_text(json.dumps(rows, indent=2) + "\n")
    plt.rcParams.update(
        {
            "font.family": "serif",
            "axes.spines.right": False,
            "axes.spines.top": False,
            "pdf.fonttype": 42,
        }
    )
    fig, ax = plt.subplots(2, 2, figsize=(8, 5.6), layout="constrained")
    for col, route in enumerate(["agile_course", TEST]):
        for row, methods in enumerate(
            [
                ["old_mpc", "wide_linear_same_mpc", "wide_nonlinear"],
                ["old_ars", "policy_initial", "ars_731", "ars_732", "ars_733"],
            ]
        ):
            labels = {
                "old_mpc": "Original linear MPC",
                "wide_linear_same_mpc": "Refit linear MPC",
                "wide_nonlinear": "Nonlinear predictor MPC",
                "old_ars": "Original ARS",
                "policy_initial": "New policy before training",
                "ars_731": "Retrained ARS (3 seeds)",
            }
            for j, method in enumerate(methods):
                t, _, _, err = traces[route, method]
                ax[row, col].plot(
                    t,
                    100 * err,
                    color=["#555555", "#ac6c2c", "#21618a"][min(j, 2)],
                    alpha=0.9 if j < 3 else 0.45,
                    lw=1.15,
                    label=labels.get(method),
                )
                record = next(r for r in rows if r["route"] == route and r["method"] == method)
                if not record["full_traversal"]:
                    ax[row, col].plot(t[-1], 100 * err[-1], marker="x", color="#ac6c2c")
                    ax[row, col].annotate(
                        "Tilt limit",
                        (t[-1], 100 * err[-1]),
                        xytext=(-36, -12),
                        textcoords="offset points",
                        fontsize=8,
                    )
            ax[row, col].set_ylabel("Position error (cm)")
            if row == 1:
                ax[row, col].set_xlabel("Simulation time (s)")
            if col == 0:
                ax[row, col].legend(frameon=False, fontsize=8)
        ax[0, col].set_title(["Previous agile course", "Held-out course"][col])
    for ext in ["png", "pdf"]:
        fig.savefig(out / f"tracking.{ext}", dpi=220)


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
