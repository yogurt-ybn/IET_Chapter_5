"""Time-weighted irregular-sample tracking/timing metrics and dwell-based recovery."""

import numpy as np


def time_rms(t, squared_error):
    if len(t) < 2 or t[-1] <= t[0]:
        return None
    return float(np.sqrt(np.trapezoid(squared_error, t) / (t[-1] - t[0])))


def settle_time(t, distance, onset, tolerance, dwell):
    candidate = None
    for ti, ei in zip(t, distance):
        if ti < onset:
            continue
        if ei <= tolerance:
            if candidate is None:
                candidate = ti
            if ti - candidate >= dwell:
                return float(candidate - onset)
        else:
            candidate = None
    return None  # Censored/non-recovery is never zero.


def compute(rows, config):
    if len(rows) < 2:
        return {"valid_samples": len(rows)}
    t = np.array([r["t"] for r in rows])
    e = np.array([r["reference"]["p"] for r in rows]) - np.array([r["truth"]["p"] for r in rows])
    norm = np.linalg.norm(e, axis=1)
    result = {"valid_samples": len(rows), "duration_s": float(t[-1] - t[0]),
              "position_rms_m": time_rms(t, np.sum(e**2, axis=1)),
              "position_max_m": float(norm.max()),
              "axis_rms_m": [time_rms(t, e[:, i]**2) for i in range(3)]}
    intervals = {"initial_hover": (0, config["transfer_onset"]),
                 "transfer": (config["transfer_onset"], config["transfer_onset"] + config["transfer_duration"]),
                 "steady_hold": (config["transfer_onset"] + config["transfer_duration"] + 1, t[-1])}
    if rows[0]["task"] == "circle":
        intervals = {"loop": (config["circle_onset"], config["circle_onset"] + config["circle_duration"])}
    for name, (start, end) in intervals.items():
        mask = (t >= start) & (t <= end)
        result[name + "_rms_m"] = time_rms(t[mask], np.sum(e[mask]**2, axis=1))
    result["transfer_settle_s"] = settle_time(t, norm,
        config["transfer_onset"] + config["transfer_duration"], config["settle_tolerance"],
        config["settle_dwell"])
    saturation = np.array([any(r["saturation"]) for r in rows], dtype=float)
    result["saturation_duration_s"] = float(np.trapezoid(saturation, t))
    result["saturation_fraction"] = result["saturation_duration_s"] / (t[-1] - t[0])
    result["allocation_residual_max"] = float(max(np.linalg.norm(r["allocation_residual"]) for r in rows))
    # This mixes N and N m only as a diagnostic norm, not an effort/energy metric.
    for name in ["controller_s", "allocation_s", "rpc_step_s", "wall_loop_s"]:
        values = [r["timing"][name] for r in rows]
        result[name] = dict(zip(["median", "p95", "max"], np.quantile(values, [.5, .95, 1]).tolist()))
    result["wall_deadline_misses"] = sum(r["timing"]["wall_loop_s"] > config["control_period"] for r in rows)
    result["fallback_count"] = sum(bool(r["diagnostics"].get("fallback", False)) for r in rows)
    result["collision_samples"] = sum(r["has_collided"] for r in rows)
    result["recovery_s"] = settle_time(t, norm, config["disturbance_onset"] + config["disturbance_duration"],
                                       config["settle_tolerance"], config["settle_dwell"])
    return result
