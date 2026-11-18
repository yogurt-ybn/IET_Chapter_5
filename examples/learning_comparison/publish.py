"""Package compact measured flight histories; plot all method traces from logged samples."""

import argparse
import hashlib
import json
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from model import read_flight

BASE = Path("results/examples/learning_case_study/runs")
TRAJECTORIES = ("figure_eight", "agile_course")
LABELS = {
    "off": "Nominal NMPC",
    "on": "GP + NMPC",
    "dmdc_mpc": "DMDc + MPC",
    "ars_policy": "ARS policy",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize(raw, output):
    model = np.load(output / "model.npz")
    records = []
    for directory in sorted(raw.iterdir()):
        result = json.loads((directory / "result.json").read_text())
        meta = json.loads((directory / "metadata.json").read_text())
        trajectory = meta["config"]["task"]
        baseline = BASE / f"nmpc-{trajectory}-nominal-r0-off"
        original = json.loads((baseline / "metadata.json").read_text())
        for key in ["parameters", "controller_parameters", "feedback", "timing_mode"]:
            assert meta[key] == original[key], key
        for key in ["p", "v", "R", "omega"]:
            assert np.allclose(
                meta["verified_initial_state"][key],
                original["verified_initial_state"][key],
                atol=1e-7,
            ), key
        assert meta["config"]["task_config"] == original["config"]["task_config"]
        rows = [json.loads(line) for line in (directory / "log.jsonl").read_text().splitlines()]
        np.savez_compressed(
            output / (directory.name + ".npz"),
            t=[r["t"] for r in rows],
            reference_p=[r["reference"]["p"] for r in rows],
            truth_p=[r["truth"]["p"] for r in rows],
            requested_wrench=[r["requested_wrench"] for r in rows],
            motor_pwm=[r["motor_pwm"] for r in rows],
        )
        records.append(
            {
                "run": directory.name,
                "status": result["status"],
                "failure": result["failure"],
                "metrics": result["metrics"],
                "log_sha256": digest(directory / "log.jsonl"),
                "physics_sha256": digest(directory / "physics.jsonl"),
            }
        )
    if len(records) != 8:
        raise ValueError("Expected eight new attempts")
    (output / "evaluation.json").write_text(json.dumps(records, indent=2) + "\n")
    # Held-out open-loop predictions on independent archived nominal NMPC trajectories.
    from scipy.spatial.transform import Rotation

    prediction = {}
    for name in TRAJECTORIES:
        raw_run = BASE / f"nmpc-{name}-nominal-r0-off"
        data = np.load(raw_run / "metrics_samples.npz")
        if np.max(np.abs(data["allocation_residual"])) > 1e-6:
            raise ValueError("Validation requires unsaturated command or achieved wrench")
        state = np.column_stack(
            (
                data["truth_p"],
                data["truth_v"],
                Rotation.from_matrix(data["truth_R"]).as_euler("xyz"),
                data["truth_omega"],
                data["telemetry_estimated_wrench"] - model["hover"],
            )
        )
        prediction[name] = {}
        for n in (1, 60):
            ids = np.arange(0, len(state) - n, n)
            x = state[ids].copy()
            for step in range(n):
                u = data["requested_wrench"][ids + step] - model["hover"]
                x = x @ model["A"].T + u @ model["B"].T + model["bias"]
            error = x[:, :3] - state[ids + n, :3]
            prediction[name][str(n)] = {
                "windows": len(ids),
                "endpoint_position_rms_m": float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
            }
    (output / "prediction.json").write_text(json.dumps(prediction, indent=2) + "\n")


def plot(output):
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 10,
            "axes.spines.right": False,
            "axes.spines.top": False,
            "pdf.fonttype": 42,
        }
    )
    fig, axes = plt.subplots(2, 2, figsize=(8.4, 5.4), layout="constrained")
    for i, name in enumerate(TRAJECTORIES):
        ax, errors = axes[:, i]
        for j, (key, label) in enumerate(LABELS.items()):
            path = (
                BASE / f"nmpc-{name}-nominal-r0-{key}" / "metrics_samples.npz"
                if key in ("off", "on")
                else output / f"{name}-{key}-r0.npz"
            )
            data = np.load(path)
            pos, ref, t = data["truth_p"], data["reference_p"], data["t"]
            if j == 0:
                ax.plot(*ref[:, :2].T, color=".7", lw=3, label="Reference")
            color = ["#505050", "#298055", "#20559b", "#a44e1d"][j]
            ax.plot(*pos[:, :2].T, lw=0.95, alpha=0.85, color=color, label=label)
            error = np.linalg.norm(pos - ref, axis=1)
            errors.plot(t, error * 1000, color=color, lw=1)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set(
            xlabel="North (m)", ylabel="West (m)", title=["Figure eight", "Agile spatial course"][i]
        )
        errors.set(xlabel="Simulation time (s)", ylabel="Position error (mm)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside lower center", ncol=5, frameon=False)
    for fmt in ("png", "pdf"):
        fig.savefig(output / f"tracking.{fmt}", dpi=230)
    plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for key in ("collection", "evaluation", "output"):
        p.add_argument(key, type=Path)
    args = p.parse_args()
    hover = np.load(args.output / "model.npz")["hover"]
    for name in ("circle", "helix", "transfer"):
        path = args.collection / name
        x, u = read_flight(path, hover)
        np.savez_compressed(
            args.output / f"identification-{name}.npz",
            state=x,
            action=u,
            log_sha256=digest(path / "log.jsonl"),
            physics_sha256=digest(path / "physics.jsonl"),
        )
    summarize(args.evaluation, args.output)
    plot(args.output)
