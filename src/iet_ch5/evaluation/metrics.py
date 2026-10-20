"""Time-weighted irregular-sample tracking/timing metrics and dwell-based recovery."""

import numpy as np
from scipy.spatial.transform import Rotation


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


def compute(rows, config, gravity):
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
    result["allocation_residual_axis_max"] = np.max(np.abs(
        [r["allocation_residual"] for r in rows]), axis=0).tolist()
    # This mixes N and N m only as a diagnostic norm, not an effort/energy metric.
    for name in ["controller_s", "allocation_s", "rpc_step_s", "wall_loop_s"]:
        values = [r["timing"][name] for r in rows]
        result[name] = dict(zip(["median", "p95", "max"], np.quantile(values, [.5, .95, 1]).tolist()))
    result["wall_deadline_misses"] = sum(r["timing"]["wall_loop_s"] > config["control_period"] for r in rows)
    result["fallback_count"] = sum(bool(r["diagnostics"].get("fallback", False)) for r in rows)
    solved_rows = [r for r in rows if r["diagnostics"].get("qp_solved_this_update", False)]
    if solved_rows:
        result["qp_updates"] = len(solved_rows)
        result["fallback_activations"] = sum(r["diagnostics"].get("fallback",False) for r in solved_rows)
        result["qp_update_computation_s"] = dict(zip(["median","p95","max"],
            np.quantile([r["timing"]["controller_s"] for r in solved_rows],[.5,.95,1]).tolist()))
        statuses = [r["diagnostics"]["solver_status"] for r in solved_rows]
        result["solver_status_counts"] = {s:statuses.count(s) for s in sorted(set(statuses))}
    result["collision_samples"] = sum(r["has_collided"] for r in rows)
    result["recovery_s"] = None
    if rows[0]["task"] == "disturbance":
        onset,end = config["disturbance_onset"],config["disturbance_onset"]+config["disturbance_duration"]
        mask = (t>=onset)&(t<end)
        result["disturbance_interval_rms_m"] = time_rms(t[mask], np.sum(e[mask]**2,axis=1))
        result["recovery_s"] = settle_time(t,norm,end,config["settle_tolerance"],config["settle_dwell"])
    # Independent truth attitude/rates against the controller's commanded target.
    attitude, rate = [], []
    for row in rows:
        ref,d = row["reference"],row["diagnostics"]
        Rd = np.array(d.get("desired_R", Rotation.from_euler("xyz", [
            -ref["a"][1]/gravity,ref["a"][0]/gravity,ref["heading"]]).as_matrix()))
        R = np.array(row["truth"]["R"])
        attitude.append(Rotation.from_matrix(Rd.T@R).magnitude())
        wd = np.array(d.get("desired_rate",[-ref["jerk"][1]/gravity,
                        ref["jerk"][0]/gravity,ref["heading_rate"]]))
        target = np.array(d["rate_target_body"]) if "rate_target_body" in d else R.T@Rd@wd
        rate.append(np.linalg.norm(np.array(row["truth"]["omega"])-target))
    result["attitude_rms_rad"] = time_rms(t,np.square(attitude))
    result["rate_rms_rad_s"] = time_rms(t,np.square(rate))
    pwm=np.array([r["motor_pwm"] for r in rows])
    result["mean_normalized_squared_motor_command"] = float(np.trapezoid(np.mean(pwm**2,axis=1),t)/(t[-1]-t[0]))
    violation = np.array([max(0.,r["constraints"]["tilt_rad"]-config.get("evaluation_tilt_limit",.3),
                        r["constraints"]["speed"]-config.get("evaluation_speed_limit",2.)) for r in rows])
    result["constraint_violation_samples"] = int(np.count_nonzero(violation>1e-6))
    result["constraint_violation_duration_s"] = float(np.trapezoid((violation>1e-6).astype(float),t))
    return result
