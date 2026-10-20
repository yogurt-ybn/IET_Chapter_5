"""Recreate all-run comparison figures from the unfiltered frozen summary."""

import argparse
import json
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

COLORS = {
    "pid": "#0072B2",
    "lqr": "#E69F00",
    "geometric": "#009E73",
    "dob": "#CC79A7",
    "mpc": "#D55E00",
}


def save(fig, path):
    fig.tight_layout()
    for ext in ["pdf", "svg", "png"]:
        fig.savefig(path.with_suffix("." + ext), dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot(source, output):
    output.mkdir(parents=True, exist_ok=True)
    font = (
        "Times New Roman"
        if "Times New Roman" in {x.name for x in font_manager.fontManager.ttflist}
        else "DejaVu Serif"
    )
    plt.rcParams.update(
        {
            "font.family": font,
            "font.size": 9,
            "svg.fonttype": "none",
            "pdf.fonttype": 42,
            "axes.grid": True,
            "grid.alpha": 0.2,
        }
    )
    groups = json.loads((source / "grouped_summary.json").read_text())
    runs = json.loads((source / "summary.json").read_text())
    methods = list(COLORS)
    fig, axes = plt.subplots(2, 3, figsize=(12, 7))
    for ax, task in zip(
        axes.flat, ["hover_transfer", "circle", "disturbance", "mismatch", "noise_delay"]
    ):
        selected = [g for g in groups if g["task"] == task]
        for i, method in enumerate(methods):
            g = next(x for x in selected if x["controller"] == method)
            v = g["rms_median_m"]
            ax.bar(i, v, color=COLORS[method], alpha=0.85)
            ax.errorbar(
                i,
                v,
                yerr=np.array([[v - g["rms_min_m"]], [g["rms_max_m"] - v]]),
                fmt="none",
                ecolor="black",
                capsize=3,
            )
            if g["failed"]:
                ax.annotate(
                    f"{g['failed']}/{g['n']} failed",
                    (i, v),
                    xytext=(0, 6),
                    textcoords="offset points",
                    ha="center",
                    fontsize=7,
                    color="darkred",
                )
        ax.set_xticks(range(5), methods, rotation=20)
        ax.set_title(task.replace("_", " "))
        ax.set_ylabel("Whole-run position RMS (m)")
        ax.margins(y=0.25)
    axes.flat[-1].axis("off")
    axes.flat[-1].text(
        0,
        0.8,
        "Bars: median across recorded runs\nWhiskers: observed min/max\nFailures retained; thresholds apply to hold/loop\nNear-hover tasks, fixed gains and one plant\nNo real-time or statistical-ranking claim",
        va="top",
        linespacing=1.8,
    )
    save(fig, output / "accuracy_matrix")
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for i, method in enumerate(methods):
        records = [r for r in runs if r["controller"] == method and r["observer_enabled"]]
        median_p95 = np.median([r["metrics"]["controller_s"]["p95"] for r in records]) * 1e3
        axes[0].bar(i, median_p95, color=COLORS[method])
        axes[1].bar(
            i,
            np.median([r["metrics"]["wall_loop_s"]["p95"] for r in records]) * 1e3,
            color=COLORS[method],
        )
        if method == "mpc":
            axes[0].scatter(
                i,
                np.median([r["metrics"]["qp_update_computation_s"]["p95"] for r in records]) * 1e3,
                marker="x",
                s=70,
                c="black",
                label="MPC QP-update p95",
            )
    for ax in axes:
        ax.set_xticks(range(5), methods, rotation=20)
    axes[0].set_ylabel("Median of per-run computation p95 (ms)")
    axes[0].legend(fontsize=8)
    axes[1].set_ylabel("Median of paused-loop wall p95 (ms)")
    axes[1].axhline(12, color="black", linestyle="--", label="12 ms nominal physics control")
    axes[1].legend(fontsize=8)
    save(fig, output / "timing_matrix")
    fig = plt.figure(figsize=(10, 4))
    ax3 = fig.add_subplot(121, projection="3d")
    ax2 = fig.add_subplot(122)
    circles = [r for r in runs if r["task"] == "circle"]
    for index, r in enumerate(circles):
        data = np.load(source / "run_records" / r["run_id"] / "metric_samples.npz")
        p, reference = data["truth_p"], data["reference_p"]
        if index == 0:
            ax3.plot(*reference.T, "k--", label="Reference")
            ax2.plot(*reference[:, :2].T, "k--", label="Reference")
        ax3.plot(*p.T, color=COLORS[r["controller"]])
        ax2.plot(*p[:, :2].T, color=COLORS[r["controller"]], label=r["controller"])
    ax3.set(xlabel="North x (m)", ylabel="West y (m)", zlabel="Up z (m)")
    ax3.set_zlim(1.85, 2.15)
    ax3.set_box_aspect((1.1, 1.1, .3))
    ax2.set(xlabel="North x (m)", ylabel="West y (m)", aspect="equal")
    ax2.legend(fontsize=8)
    save(fig, output / "circle_trajectories")
    fig, ax = plt.subplots(figsize=(8, 4))
    for r in circles:
        data = np.load(source / "run_records" / r["run_id"] / "metric_samples.npz")
        ax.plot(
            data["t"],
            np.linalg.norm(data["reference_p"] - data["truth_p"], axis=1),
            color=COLORS[r["controller"]],
            label=r["controller"],
        )
    ax.axvspan(2, 16, color="gray", alpha=0.12)
    ax.set(xlabel="Simulation time (s)", ylabel="Position error norm (m)")
    ax.legend(ncol=5)
    save(fig, output / "circle_errors")
    fig, ax = plt.subplots(figsize=(8, 4))
    for seed in range(5):
        paired = [
            next(
                r
                for r in runs
                if r["task"] == "noise_delay" and r["seed"] == seed and r["controller"] == method
            )["metrics"]["position_rms_m"]
            for method in methods
        ]
        ax.plot(range(5), paired, color="gray", alpha=0.4, linewidth=0.7)
        for i, method in enumerate(methods):
            ax.scatter(
                i, paired[i], color=COLORS[method], marker=["o", "s", "^", "D", "v"][seed], s=35
            )
    ax.set_xticks(range(5), methods)
    ax.set_ylabel("Position RMS (m), five paired noise/delay seeds")
    save(fig, output / "paired_noise_distribution")
    (output / "provenance.json").write_text(
        json.dumps(
            {
                "source": "scripts/plot_benchmark_summary.py",
                "summary": "results/examples/m3/summary.json",
                "font": font,
                "failures_included": True,
            },
            indent=2,
        )
        + "\n"
    )
    print("Regenerated matrix figures without excluding failures")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    plot(a.source, a.output)
