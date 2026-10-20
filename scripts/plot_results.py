#!/usr/bin/env python3
"""Regenerate editable-source publication figures from actual common JSONL logs."""

import argparse
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np


COLORS = {"pid": "#0072B2", "lqr": "#E69F00", "geometric": "#009E73",
          "dob": "#CC79A7", "mpc": "#D55E00"}


def save(fig, output, name):
    fig.tight_layout()
    for ext in ["pdf", "svg", "png"]:
        fig.savefig(output / f"{name}.{ext}", dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot(run_paths, output):
    output.mkdir(parents=True, exist_ok=True)
    available = {f.name for f in font_manager.fontManager.ttflist}
    font = "Times New Roman" if "Times New Roman" in available else "DejaVu Serif"
    plt.rcParams.update({"font.family": font, "font.size": 10, "axes.grid": True,
                         "grid.alpha": .25, "svg.fonttype": "none", "pdf.fonttype": 42})
    runs = []
    for path in run_paths:
        log = path / "log.jsonl"
        content = log.read_text() if log.exists() else gzip.decompress((path / "log.jsonl.gz").read_bytes()).decode()
        rows = [json.loads(line) for line in content.splitlines()]
        if not rows:
            continue
        result = json.loads((path / "result.json").read_text())
        runs.append((path, rows, result))
    if not runs:
        raise ValueError("No valid logged samples to plot")
    fig = plt.figure(figsize=(10, 4))
    ax3 = fig.add_subplot(121, projection="3d")
    ax2 = fig.add_subplot(122)
    for index, (path, rows, result) in enumerate(runs):
        p = np.array([r["truth"]["p"] for r in rows])
        ref = np.array([r["reference"]["p"] for r in rows])
        name = result["controller"]
        label = f"{name} ({result['status']})"
        if index == 0:
            ax3.plot(*ref.T, "k--", label="Reference")
            ax2.plot(*ref[:, :2].T, "k--", label="Reference")
        ax3.plot(*p.T, color=COLORS[name], label=label)
        ax2.plot(*p[:, :2].T, color=COLORS[name], label=label)
    ax3.set(xlabel="North x (m)", ylabel="West y (m)", zlabel="Up z (m)")
    ax2.set(xlabel="North x (m)", ylabel="West y (m)", aspect="equal")
    ax2.legend(fontsize=8)
    save(fig, output, "trajectories")
    fig, axes = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
    for path, rows, result in runs:
        t = np.array([r["t"] for r in rows])
        e = np.array([r["reference"]["p"] for r in rows]) - np.array([r["truth"]["p"] for r in rows])
        for i in range(3):
            axes[i].plot(t, e[:, i], color=COLORS[result["controller"]], label=result["controller"])
        axes[3].plot(t, np.linalg.norm(e, axis=1), color=COLORS[result["controller"]])
    config = json.loads((runs[0][0] / "metadata.json").read_text())["config"]["task_config"]
    for ax, label in zip(axes, ["North error (m)", "West error (m)", "Up error (m)", "Norm (m)"]):
        ax.set_ylabel(label)
        if runs[0][1][0]["task"] == "circle":
            ax.axvspan(config["circle_onset"], config["circle_onset"] + config["circle_duration"],
                       color="gray", alpha=.1)
        else:
            ax.axvspan(config["transfer_onset"], config["transfer_onset"] + config["transfer_duration"],
                       color="gray", alpha=.1)
        if runs[0][1][0]["task"] == "disturbance":
            ax.axvspan(config["disturbance_onset"], config["disturbance_onset"] + config["disturbance_duration"],
                       color="orange", alpha=.15)
    axes[0].legend(ncol=len(runs), fontsize=8)
    axes[-1].set_xlabel("Simulation time (s)")
    save(fig, output, "position_errors")
    for path, rows, result in runs:
        t = [r["t"] for r in rows]
        requested = np.array([r["requested_wrench"] for r in rows])
        estimated = np.array([r["telemetry_estimated_wrench"] for r in rows])
        fig, axes = plt.subplots(4, 1, figsize=(8, 7), sharex=True)
        for i, ax in enumerate(axes):
            ax.plot(t, requested[:, i], label="Requested", color=COLORS[result["controller"]])
            ax.plot(t, estimated[:, i], "k--", lw=.8, label="Telemetry model estimate")
            ax.set_ylabel(["Thrust (N)", "Roll (N m)", "Pitch (N m)", "Yaw (N m)"][i])
        axes[0].legend(fontsize=8)
        axes[-1].set_xlabel("Simulation time (s)")
        save(fig, output, result["controller"] + "_wrench")
        fig, axes = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
        pwm = np.array([r["motor_pwm"] for r in rows])
        for i, rotor in enumerate(["Front right", "Rear left", "Front left", "Rear right"]):
            axes[0].plot(t, pwm[:, i], label=rotor)
        axes[0].set(ylabel="Normalized input", ylim=(-.03, 1.03))
        axes[0].legend(ncol=4, fontsize=8)
        axes[1].plot(t, [sum(r["saturation"]) for r in rows])
        axes[1].set(ylabel="Saturated rotors", xlabel="Simulation time (s)", ylim=(-.1, 4.1))
        save(fig, output, result["controller"] + "_rotors")
    fig, axes = plt.subplots(1, 2, figsize=(9, 4))
    labels = [result["controller"] for _, _, result in runs]
    for i, (_, rows, result) in enumerate(runs):
        values = 1e3 * np.array([r["timing"]["controller_s"] for r in rows])
        axes[0].boxplot(values, positions=[i], widths=.55, showfliers=False)
        axes[1].bar(i, 1e3*result["metrics"]["wall_loop_s"]["p95"], color=COLORS[result["controller"]])
    for ax in axes:
        ax.set_xticks(range(len(labels)), labels)
    axes[0].set_ylabel("Controller computation (ms)")
    axes[1].set_ylabel("Paused-loop wall p95 (ms)")
    save(fig, output, "timing")
    summary = [{"run": path.name, "controller": result["controller"], "status": result["status"],
                **result["metrics"]} for path, _, result in runs]
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (output / "plot_provenance.json").write_text(json.dumps({"runs": [str(p) for p, _, _ in runs],
        "font": font, "source": "scripts/plot_results.py", "evidence": "actual AirSim logs"}, indent=2) + "\n")
    print(f"Regenerated figures for {len(runs)} logged runs; font={font}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("runs", type=Path, nargs="+")
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    plot(args.runs, args.output)
