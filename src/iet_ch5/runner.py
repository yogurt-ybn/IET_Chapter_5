"""Common simulation-time experiment runner with independent truth evaluation and fail logs."""

import json
import time
import traceback
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .allocation import Allocator
from .backends.airsim_backend import AirSimBackend
from .controllers import make_controller
from .evaluation.metrics import compute
from .logging import config_hash, dump, revision
from .references.tasks import Task
from .types import RunResult, WrenchCommand


def run(local_config, controllers_config, task_config, controller_id, task_name, output,
        *, seed=0):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    c = local_config
    backend = AirSimBackend(c["host"], c["port"], c["vehicle"], exclusive=True)
    allocator = Allocator(backend.parameters)
    controller = make_controller(controller_id, controllers_config[controller_id])
    task = Task(task_name, task_config)
    config = {"controller_id": controller_id, "controllers": controllers_config,
              "task": task_name, "task_config": task_config, "seed": seed}
    metadata = {"schema": "iet-ch5/v1", "run_id": output.name, "config": config,
                "config_sha256": config_hash(config), "software": revision(),
                "parameters": asdict(backend.parameters),
                "backend_provenance": json.loads(Path("docs/airsim_source_inventory.json").read_text()),
                "feedback": "atomic physics truth", "timing_mode": "paused simulation-time",
                "reference_information": "p,v,a,jerk,snap,heading and derivatives; no preview at M1"}
    (output / "metadata.json").write_text(dump(metadata) + "\n")
    rows = []
    status, failure = "failed", None
    try:
        backend.initialize(task_config["initial_position"])
        hover = allocator.allocate(WrenchCommand(
            backend.parameters.mass * backend.parameters.gravity, np.zeros(3)))
        state, samples, _ = backend.step(hover.pwm, task_config["control_period"])
        controller.reset(state, task.at(0), backend.parameters)
        initial_timestamp = state.timestamp_ns
        previous_timestamp = initial_timestamp - int(task_config["control_period"] * 1e9)
        duration = task_config["circle_total_duration"] if task_name == "circle" else task_config["duration"]
        with (output / "log.jsonl").open("w") as log, (output / "physics.jsonl").open("w") as physics:
            while (state.timestamp_ns - initial_timestamp) * 1e-9 < duration:
                wall_start = time.perf_counter()
                dt = (state.timestamp_ns - previous_timestamp) * 1e-9
                if not 0 < dt < .1:
                    raise RuntimeError(f"Invalid control sample interval {dt}")
                t = (state.timestamp_ns - initial_timestamp) * 1e-9
                reference = task.at(t)
                start = time.perf_counter()
                command, diagnostics = controller.update(state, reference, dt)
                computation = time.perf_counter() - start
                start = time.perf_counter()
                allocated = allocator.allocate(command)
                allocation_time = time.perf_counter() - start
                applied = allocator.from_telemetry(samples[-1]["rotors"])
                if hasattr(controller, "observe_applied"):
                    controller.observe_applied(applied, allocated.residual, dt)
                row = {"schema": "iet-ch5/v1", "run_id": output.name,
                       "config_sha256": metadata["config_sha256"], "controller": controller_id,
                       "task": task_name, "t": t, "simulation_timestamp_ns": state.timestamp_ns,
                       "monotonic_ns": time.monotonic_ns(), "dt": dt,
                       "reference": asdict(reference), "truth": asdict(state), "feedback": asdict(state),
                       "requested_wrench": command.vector(),
                       "steady_predicted_wrench": allocated.predicted_wrench,
                       "telemetry_estimated_wrench": applied,
                       "motor_pwm": allocated.pwm, "motor_states": samples[-1]["rotors"],
                       "allocation_residual": allocated.residual, "saturation": allocated.saturated,
                       "has_collided": samples[-1]["has_collided"], "disturbance": None,
                       "constraints": {"position_error": np.linalg.norm(reference.p - state.p),
                           "tilt_rad": np.arccos(np.clip(state.R[2, 2], -1, 1)),
                           "speed": np.linalg.norm(state.v)}, "diagnostics": diagnostics,
                       "timing": {"controller_s": computation, "allocation_s": allocation_time}}
                previous_timestamp = state.timestamp_ns
                state, samples, rpc_time = backend.step(allocated.pwm, task_config["control_period"])
                for sample in samples:
                    physics.write(dump(sample) + "\n")
                row["timing"].update(rpc_step_s=rpc_time, wall_loop_s=time.perf_counter() - wall_start)
                encoded = dump(row)
                log.write(encoded + "\n")
                rows.append(json.loads(encoded))
                if (row["constraints"]["position_error"] > task_config["abort_position_error"]
                    or row["constraints"]["tilt_rad"] > task_config["abort_tilt_rad"]
                    or row["constraints"]["speed"] > task_config["abort_speed"]
                    or row["has_collided"]):
                    raise RuntimeError("Safety envelope/collision abort; failed samples retained")
        metrics = compute(rows, task_config)
        error = metrics.get("steady_hold_rms_m") if task_name != "circle" else metrics["loop_rms_m"]
        status = "passed" if error is not None and error < task_config["hold_acceptance_rms_m"] else "failed"
        if status == "failed":
            failure = "Tracking acceptance tolerance not met"
    except Exception as error:
        failure = f"{type(error).__name__}: {error}"
        (output / "failure.txt").write_text(traceback.format_exc())
    finally:
        backend.stop()
        backend.close()
    result = RunResult(output.name, controller_id, status, str(output / "log.jsonl"),
                       compute(rows, task_config), failure)
    (output / "result.json").write_text(dump(result) + "\n")
    return result
