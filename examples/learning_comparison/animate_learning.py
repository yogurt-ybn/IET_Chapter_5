"""Render a synchronised nominal agile-course replay from committed AirSim logs."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

ROOT = Path(__file__).resolve().parents[2] / "results/examples"
GP = ROOT / "learning_case_study/runs"
WIDE = ROOT / "learning_retrained"
RUNS = [
    ("Nominal NMPC", GP / "nmpc-agile_course-nominal-r0-off/metrics_samples.npz", "#475569"),
    ("GP residual + NMPC", GP / "nmpc-agile_course-nominal-r0-on/metrics_samples.npz", "#00785c"),
    ("Learned dynamics + MPC", WIDE / "agile_course-wide_nonlinear.npz", "#205fac"),
    ("ARS policy (seed 731)", WIDE / "agile_course-ars_731.npz", "#af4a18"),
]


def main():
    fig = plt.figure(figsize=(10.4, 4.8), dpi=95, layout="constrained")
    grid = fig.add_gridspec(2, 4, height_ratios=[1.25, 1])
    errors = fig.add_subplot(grid[1, :])
    traces = []
    for col, (name, path, color) in enumerate(RUNS):
        with np.load(path) as data:
            t, p, ref = (data[key] for key in ("t", "truth_p", "reference_p"))
        if traces:
            ref_on_grid = np.column_stack([np.interp(t, traces[0][0], col) for col in shared_ref.T])
            np.testing.assert_allclose(ref, ref_on_grid, atol=0.0005, rtol=0)
        else:
            shared_ref = ref
        error = 1000 * np.linalg.norm(p - ref, axis=1)
        ax = fig.add_subplot(grid[0, col])
        ax.set_title(name, fontsize=10, fontweight="bold", color=color)
        ax.plot(*ref[:, :2].T, color="#acb4bc", lw=2.8)
        (track,) = ax.plot([], [], color=color, lw=1.2)
        (dot,) = ax.plot([], [], "o", color=color, ms=5)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("North (m)", fontsize=9)
        if col == 0:
            ax.set_ylabel("West (m)", fontsize=9)
        (line,) = errors.plot([], [], color=color, lw=1.4, label=name)
        traces.append((t, p, error, track, dot, line))
    errors.set(
        xlabel="Simulation time (s)",
        ylabel="3D position error (mm)",
        xlim=(0, t[-1]),
        ylim=(0, max(tr[2].max() for tr in traces) * 1.08),
    )
    errors.legend(loc="upper left", ncols=2, frameon=False, fontsize=8)
    errors.grid(axis="y", alpha=0.18)
    for ax in fig.axes:
        ax.spines[["top", "right"]].set_visible(False)
    title = fig.suptitle("AirSim agile course", fontsize=13)
    end = min(item[0][-1] for item in traces)
    frames = np.r_[np.arange(0, end, 0.1), end]

    def draw(stamp):
        title.set_text(f"AirSim agile course   |   {stamp:4.1f} s")
        for times, xyz, err, track, dot, line in traces:
            index = np.searchsorted(times, stamp, side="right") - 1
            track.set_data(xyz[: index + 1, 0], xyz[: index + 1, 1])
            dot.set_data([xyz[index, 0]], [xyz[index, 1]])
            line.set_data(times[: index + 1], err[: index + 1])
        return []

    movie = FuncAnimation(fig, draw, frames=frames, repeat=True)
    movie.save(WIDE / "learning_comparison.gif", writer=PillowWriter(fps=10))
    plt.close(fig)


if __name__ == "__main__":
    main()
