"""Postprocess the frozen AirSim matrix without changing or filtering its recorded outcomes."""

import argparse
import csv
import gzip
import hashlib
import json
import shutil
from pathlib import Path
import numpy as np
from iet_ch5.evaluation.reanalyze import reanalyze as REANALYZE


def load(path):
    return json.loads(path.read_text())


def write(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n")


def log_rows(path):
    file = path / "log.jsonl"
    raw = (
        file.read_bytes()
        if file.exists()
        else gzip.decompress((path / "log.jsonl.gz").read_bytes())
    )
    return raw, [json.loads(s) for s in raw.decode().splitlines()]


def build(source, output):
    SOURCE, OUT = Path(source), Path(output)
    results = load(SOURCE / "summary.json")
    assert len(results) == 51, "Final matrix is unfinished"
    OUT.mkdir(parents=True, exist_ok=True)
    write(OUT / "summary_recorded_original.json", results)
    for record in results:
        destination = OUT / "run_records" / record["run_id"]
        destination.mkdir(parents=True, exist_ok=True)
        common, _ = REANALYZE(SOURCE / record["run_id"], destination / "metric_samples.npz")
        record["metrics_recorded_original"] = record["metrics"].copy()
        record["metrics"] = {**record["metrics"], **common}
        error = record["metrics"].get(
            "loop_rms_m" if record["task"] == "circle" else "steady_hold_rms_m"
        )
        record["common_interval_tracking_passed"] = bool(
            common.get("complete_common_interval") and error is not None and error < 0.1
        )
    write(OUT / "summary.json", results)
    shutil.copy2(SOURCE / "manifest.json", OUT / "manifest.json")
    groups = []
    methods = ["pid", "lqr", "geometric", "dob", "mpc"]
    for task in ["hover_transfer", "circle", "disturbance", "mismatch", "noise_delay"]:
        for method in methods:
            runs = [
                r
                for r in results
                if r["controller"] == method and r["task"] == task and r["observer_enabled"]
            ]
            values = [
                r["metrics"]["position_rms_m"] for r in runs if "position_rms_m" in r["metrics"]
            ]
            groups.append(
                {
                    "task": task,
                    "controller": method,
                    "n": len(runs),
                    "passed": sum(r["status"] == "passed" for r in runs),
                    "failed": sum(r["status"] != "passed" for r in runs),
                    "rms_median_m": float(np.median(values)) if values else None,
                    "rms_min_m": min(values) if values else None,
                    "rms_max_m": max(values) if values else None,
                    "whole_or_partial_duration": "Failed/aborted outcomes retained; inspect per-run duration before comparing",
                }
            )
    write(OUT / "grouped_summary.json", groups)
    with (OUT / "summary.csv").open("w") as f:
        columns = [
            "run_id",
            "controller",
            "task",
            "seed",
            "repeat",
            "observer_enabled",
            "status",
            "failure",
            "position_rms_m",
            "position_max_m",
            "steady_hold_rms_m",
            "loop_rms_m",
            "disturbance_interval_rms_m",
            "recovery_s",
            "saturation_fraction",
            "constraint_violation_samples",
            "fallback_count",
            "fallback_activations",
            "wall_deadline_misses",
            "attitude_rms_rad",
            "rate_rms_rad_s",
            "mean_normalized_squared_motor_command",
        ]
        writer = csv.DictWriter(f, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for r in results:
            row = {**r, **r["metrics"]}
            writer.writerow({k: row.get(k) for k in columns})
    repeats = []
    for method in methods:
        r = [r for r in results if r["controller"] == method and r["task"] == "hover_transfer"]
        paths = [SOURCE / x["run_id"] for x in r]
        a, b = [log_rows(p)[1] for p in paths]
        grid = np.arange(0, min(a[-1]["t"], b[-1]["t"]), 0.012)
        positions = []
        for rows in [a, b]:
            t = np.array([v["t"] for v in rows])
            p = np.array([v["truth"]["p"] for v in rows])
            positions.append(np.column_stack([np.interp(grid, t, p[:, i]) for i in range(3)]))
        max_delta = float(np.max(np.linalg.norm(positions[0] - positions[1], axis=1)))
        rms_delta = abs(r[0]["metrics"]["position_rms_m"] - r[1]["metrics"]["position_rms_m"])
        repeats.append(
            {
                "controller": method,
                "runs": [v["run_id"] for v in r],
                "resampling_dt_s": 0.012,
                "max_paired_position_delta_m": max_delta,
                "rms_metric_delta_m": rms_delta,
                "declared_tolerances": {"max_position_delta_m": 0.02, "rms_metric_delta_m": 0.002},
                "passed": max_delta < 0.02 and rms_delta < 0.002,
            }
        )
    write(OUT / "repeatability.json", repeats)
    # Check exact paired noise arrays, not just seed labels.
    pairing = []
    for seed in range(5):
        r = [r for r in results if r["task"] == "noise_delay" and r["seed"] == seed]
        arrays = [np.load(SOURCE / x["run_id"] / "noise_realization.npz")["noise"] for x in r]
        pairing.append(
            {
                "seed": seed,
                "methods": [x["controller"] for x in r],
                "arrays_identical": all(np.array_equal(arrays[0], x) for x in arrays[1:]),
                "array_sha256": hashlib.sha256(arrays[0].tobytes()).hexdigest(),
                "shape": list(arrays[0].shape),
            }
        )
    write(OUT / "noise_pairing.json", pairing)
    # One compact full-control-cadence example per controller plus observer ablation.
    selected = [
        r
        for r in results
        if (r["task"] == "hover_transfer" and r["repeat"] == 0)
        or (r["task"] == "disturbance" and r["controller"] == "dob")
    ]
    for r in selected:
        source = SOURCE / r["run_id"]
        dest = OUT / "runs" / r["run_id"]
        dest.mkdir(parents=True, exist_ok=True)
        raw, _ = log_rows(source)
        with gzip.GzipFile(filename=str(dest / "log.jsonl.gz"), mode="wb", mtime=0) as z:
            z.write(raw)
        for name in ["metadata.json", "result.json"]:
            shutil.copy2(source / name, dest / name)
        if (source / "failure.txt").exists():
            shutil.copy2(source / "failure.txt", dest / "failure.txt")
        write(
            dest / "archive.json",
            {
                "log_sha256": hashlib.sha256(raw).hexdigest(),
                "sample_selection": "all control-cadence samples; lossless gzip",
                "source_run": r["run_id"],
                "complete_raw_directory": "results/runs/frozen-m3-20261020/" + r["run_id"],
            },
        )
    # Store every per-run metadata/result and checksums, including failures; larger raw logs stay local.
    inventory = []
    for r in results:
        source = SOURCE / r["run_id"]
        dest = OUT / "run_records" / r["run_id"]
        dest.mkdir(parents=True, exist_ok=True)
        for name in ["metadata.json", "result.json", "failure.txt"]:
            if (source / name).exists():
                shutil.copy2(source / name, dest / name)
        inventory.append(
            {
                "run": r["run_id"],
                "files": {
                    p.name: {
                        "bytes": p.stat().st_size,
                        "sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
                    }
                    for p in source.iterdir()
                    if p.is_file()
                },
            }
        )
    write(OUT / "full_run_inventory.json", inventory)
    write(
        OUT / "unavailable_tests.json",
        {
            "prescribed_force_moment_pulse": "unavailable: installed RPC has no verified arbitrary external-wrench injection",
            "unpaused_real_time": "not evaluated; no real-time feasibility claim",
            "physical_flight": "outside scope",
            "external_large_archive_upload": "not uploaded; complete run directories retained locally and rerunnable",
        },
    )
    print(
        "Built all-run records without removing failures:",
        len(results),
        "runs;",
        sum(r["status"] == "passed" for r in results),
        "passed",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    build(args.source, args.output)
