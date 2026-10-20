"""Common exact-duration metric resampling from independent physics truth.

Original controller result files remain immutable. This postprocessor includes the terminal
truth sample (which the control-decision log does not contain) and uses a common grid.
"""

import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from iet_ch5.references.tasks import Task
from iet_ch5.evaluation.metrics import time_rms, settle_time


def reanalyze(path, output=None):
    path = Path(path)
    meta = json.loads((path / "metadata.json").read_text())
    control = [json.loads(s) for s in (path / "log.jsonl").read_text().splitlines()]
    physical = [json.loads(s) for s in (path / "physics.jsonl").read_text().splitlines()]
    c = meta["config"]["task_config"]
    task = meta["config"]["task"]
    start = meta["verified_initial_state"]
    stamp = start["timestamp_ns"]
    duration = c["circle_total_duration"] if task == "circle" else c["duration"]
    if not physical or not control:
        return {"valid_samples": 0, "complete_common_interval": False}, None
    # simSetKinematics emits a setup snapshot at the same paused timestamp as verified_initial_state.
    # Keep the verified initial point once for interpolation; the raw log remains untouched.
    redundant_setup = bool(physical and physical[0]["timestamp"] == stamp)
    if redundant_setup:
        physical = physical[1:]
    timestamps = np.r_[stamp, np.array([r["timestamp"] for r in physical], dtype=np.int64)]
    time = (timestamps - stamp) * 1e-9
    target_end = min(duration, float(time[-1]))
    complete = target_end >= duration
    grid = np.arange(0, target_end, c["control_period"])
    if len(grid) == 0 or target_end > grid[-1] + 1e-9:
        grid = np.r_[grid, target_end]

    def vec(v):
        return np.array([v["x_val"], -v["y_val"], -v["z_val"]])

    p = np.vstack([start["p"]] + [vec(r["kinematics"]["position"]) for r in physical])
    v = np.vstack([start["v"]] + [vec(r["kinematics"]["linear_velocity"]) for r in physical])
    omega = np.vstack(
        [start["omega"]] + [vec(r["kinematics"]["angular_velocity"]) for r in physical]
    )

    def interp(x):
        return np.column_stack([np.interp(grid, time, x[:, i]) for i in range(3)])

    p, v, omega = map(interp, [p, v, omega])
    q = [Rotation.from_matrix(start["R"]).as_quat()]
    for r in physical:
        a = r["kinematics"]["orientation"]
        q.append([a["x_val"], -a["y_val"], -a["z_val"], a["w_val"]])
    R = Slerp(time, Rotation.from_quat(q))(grid).as_matrix()
    reference = Task(task, c)
    refs = [reference.at(float(t)) for t in grid]
    pref = np.array([r.p for r in refs])
    error = pref - p
    distance = np.linalg.norm(error, axis=1)
    result = {
        "metric_processing": "independent physics truth, linear p/v/rate interpolation, attitude SLERP, common simulation-time grid including terminal endpoint",
        "redundant_setup_snapshot": redundant_setup,
        "complete_common_interval": bool(complete),
        "duration_s": float(target_end),
        "valid_interval_s": [0.0, float(target_end)],
        "metric_grid_dt_s": c["control_period"],
        "metric_grid_samples": len(grid),
        "position_rms_m": time_rms(grid, np.sum(error**2, axis=1)),
        "position_max_m": float(distance.max()),
        "axis_rms_m": [time_rms(grid, error[:, i] ** 2) for i in range(3)],
    }
    intervals = {
        "initial_hover": (0, c["transfer_onset"]),
        "transfer": (c["transfer_onset"], c["transfer_onset"] + c["transfer_duration"]),
        "steady_hold": (c["transfer_onset"] + c["transfer_duration"] + 1, target_end),
    }
    if task == "circle":
        intervals = {"loop": (c["circle_onset"], c["circle_onset"] + c["circle_duration"])}
    for name, (a, b) in intervals.items():
        # Include interpolated exact interval endpoints instead of choosing a different grid boundary.
        if b <= a or target_end < a:
            result[name + "_rms_m"] = None
            continue
        b = min(b, target_end)
        inner = grid[(grid > a) & (grid < b)]
        t = np.r_[a, inner, b]
        e = np.column_stack([np.interp(t, grid, error[:, i]) for i in range(3)])
        result[name + "_rms_m"] = time_rms(t, np.sum(e**2, axis=1))
    result["transfer_settle_s"] = settle_time(
        grid,
        distance,
        c["transfer_onset"] + c["transfer_duration"],
        c["settle_tolerance"],
        c["settle_dwell"],
    )
    result["recovery_s"] = None
    if task == "disturbance":
        a, b = c["disturbance_onset"], c["disturbance_onset"] + c["disturbance_duration"]
        mask = (grid >= a) & (grid <= b)
        result["disturbance_interval_rms_m"] = time_rms(
            grid[mask], np.sum(error[mask] ** 2, axis=1)
        )
        result["recovery_s"] = settle_time(
            grid, distance, b, c["settle_tolerance"], c["settle_dwell"]
        )
    tc = np.array([r["t"] for r in control])
    index = np.maximum(0, np.searchsorted(tc, grid, side="right") - 1)
    rd = []
    rates = []
    rate_is_body = []
    g = meta["parameters"]["gravity"]
    for r in control:
        ref, d = r["reference"], r["diagnostics"]
        rd.append(
            d.get(
                "desired_R",
                Rotation.from_euler(
                    "xyz", [-ref["a"][1] / g, ref["a"][0] / g, ref["heading"]]
                ).as_matrix(),
            )
        )
        rate_is_body.append("rate_target_body" in d)
        rates.append(
            d.get(
                "rate_target_body",
                d.get(
                    "desired_rate", [-ref["jerk"][1] / g, ref["jerk"][0] / g, ref["heading_rate"]]
                ),
            )
        )
    rd = np.array(rd)[index]
    rates = np.array(rates)[index]
    aerror = Rotation.from_matrix(np.swapaxes(rd, 1, 2) @ R).magnitude()
    target_body = np.einsum("nij,nj->ni", np.swapaxes(R, 1, 2) @ rd, rates)
    target_body[np.array(rate_is_body)[index]] = rates[np.array(rate_is_body)[index]]
    result["attitude_rms_rad"] = time_rms(grid, aerror**2)
    result["rate_rms_rad_s"] = time_rms(grid, np.sum((omega - target_body) ** 2, axis=1))
    pwm = np.array([r["motor_pwm"] for r in control])[index]
    sat = np.array([any(r["saturation"]) for r in control])[index]
    result["mean_normalized_squared_motor_command"] = float(
        np.trapezoid(np.mean(pwm**2, axis=1), grid) / target_end
    )
    result["saturation_duration_s"] = float(np.trapezoid(sat.astype(float), grid))
    result["saturation_fraction"] = result["saturation_duration_s"] / target_end
    violation = (np.arccos(np.clip(R[:, 2, 2], -1, 1)) > 0.3 + 1e-6) | (
        np.linalg.norm(v, axis=1) > 2.0 + 1e-6
    )
    result["constraint_violation_samples"] = int(violation.sum())
    result["constraint_violation_duration_s"] = float(np.trapezoid(violation.astype(float), grid))
    data = {
        "t": grid,
        "truth_p": p,
        "reference_p": pref,
        "attitude_error_rad": aerror,
        "rate_error_rad_s": np.linalg.norm(omega - target_body, axis=1),
    }
    if output is not None:
        np.savez_compressed(output, **data)
    return result, data
