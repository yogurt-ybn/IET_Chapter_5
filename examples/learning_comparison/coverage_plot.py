"""Show measured training coverage and horizon-matched prediction errors."""

import sys, json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from split_routes import TRAIN, TEST
from wide_report import envelope


def draw(raw, out):
    fig, axes = plt.subplots(1, 2, figsize=(8.8, 3.3), layout="constrained")
    plt.rcParams["pdf.fonttype"] = 42
    for names, label, color in [
        (TRAIN, "Additional training flights", "#9ecae1"),
        (("circle", "helix", "transfer"), "Original training flights", "#585858"),
    ]:
        v = np.vstack([envelope(np.load(out / f"fit-{name}.npz")["state"]) for name in names])
        axes[0].scatter(v[::4, 0], v[::4, 1], s=4, alpha=0.55, c=color, label=label, linewidths=0)
    rows = [
        json.loads(l)
        for l in (raw / "evaluation" / f"{TEST}-wide_nonlinear" / "log.jsonl")
        .read_text()
        .splitlines()
    ]
    axes[0].plot(
        [r["constraints"]["speed"] for r in rows],
        [np.degrees(r["constraints"]["tilt_rad"]) for r in rows],
        color="#a65328",
        lw=0.85,
        label="Held-out flight",
    )
    axes[0].set(xlabel="Speed (m/s)", ylabel="Tilt (degrees)")
    axes[0].legend(frameon=False, fontsize=8)
    data = json.loads((out / "validation.json").read_text())
    for i, (key, label, c) in enumerate(
        [
            ("old", "Original linear", "#555555"),
            ("wide", "Refit linear", "#ae692f"),
            ("nonlinear", "Nonlinear", "#23658c"),
        ]
    ):
        axes[1].bar(
            np.arange(2) + (i - 1) * 0.24,
            [100 * data[key][k]["position_m"] for k in ["60", "72"]],
            width=0.23,
            label=label,
            color=c,
        )
    axes[1].set_xticks([0, 1], ["0.720 s", "0.864 s"])
    axes[1].set(xlabel="Open-loop prediction horizon", ylabel="Endpoint position RMS (cm)")
    axes[1].legend(frameon=False, fontsize=8)
    for a in axes:
        a.spines[["top", "right"]].set_visible(False)
    for ext in ("png", "pdf"):
        fig.savefig(out / f"coverage.{ext}", dpi=220)


if __name__ == "__main__":
    draw(Path(sys.argv[1]), Path(sys.argv[2]))
