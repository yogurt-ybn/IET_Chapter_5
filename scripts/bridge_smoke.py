#!/usr/bin/env python3
"""M0 motor, timing and axis-sign acceptance on an explicitly exclusive simulator."""

import argparse
import json
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

from iet_ch5.allocation import Allocator
from iet_ch5.backends.airsim_backend import AirSimBackend
from iet_ch5.types import WrenchCommand


def smoke(config, check_overflow=False):
    b = AirSimBackend(config["host"], config["port"], config["vehicle"], exclusive=True)
    a = Allocator(b.parameters)
    evidence = {"parameters": asdict(b.parameters), "allocation_pwm_to_wrench": a.B,
                "tests": {}, "all_physics_samples": {}}
    mg = b.parameters.mass * b.parameters.gravity
    try:
        for label, moments in [("hover", np.zeros(3))] + [
                (f"positive_{axis}", np.eye(3)[i] * .01) for i, axis in enumerate("xyz")]:
            b.initialize()
            rows, walls = [], []
            # Prime at hover before each small independent positive moment perturbation.
            hover = a.allocate(WrenchCommand(mg, np.zeros(3))).pwm
            for _ in range(6):
                state, samples, wall = b.step(hover, .012)
                rows += samples
            start = state
            command = a.allocate(WrenchCommand(mg, moments))
            for _ in range(8):
                state, samples, wall = b.step(command.pwm, .012)
                rows += samples
                walls.append(wall)
            ts = np.array([r["timestamp"] for r in rows], dtype=np.int64)
            rotor = rows[-1]["rotors"]
            measured_input = np.array([r["control_signal_input"] for r in rotor])
            filtered = np.array([r["control_signal_filtered"] for r in rotor])
            thrust = np.array([r["thrust"] for r in rotor])
            result = {"start_p": start.p, "end_p": state.p, "delta_v": state.v - start.v,
                      "delta_body_rate": state.omega - start.omega,
                      "command_pwm": command.pwm, "observed_pwm": measured_input,
                      "filtered_pwm": filtered, "telemetry_thrust": thrust,
                      "input_mapping_error_N": np.max(np.abs(
                          thrust - filtered * b.parameters.max_thrust)),
                      "estimated_wrench": a.from_telemetry(rotor),
                      "physics_dt_s": np.diff(ts) * 1e-9,
                      "step_wall_s": walls, "samples": len(rows)}
            if label == "hover":
                result["passed"] = bool(np.linalg.norm(state.v - start.v) < .05
                                         and np.linalg.norm(state.omega) < .01)
            else:
                axis = "xyz".index(label[-1])
                result["passed"] = bool((state.omega - start.omega)[axis] > .01)
            evidence["tests"][label] = result
            evidence["all_physics_samples"][label] = rows
        # A paused timestamp remains fixed across a real wall-clock wait.
        first = b.rpc.call("getDynamicsStateBatch", b.last_sequence, 4096, b.vehicle)
        time.sleep(.03)
        second = b.rpc.call("getDynamicsStateBatch", b.last_sequence, 4096, b.vehicle)
        evidence["tests"]["pause"] = {"passed": not first["samples"] and not second["samples"]}
        # Observe lag and expired-goal persistence without installing another motor command.
        b.initialize()
        pwm = np.array([.56, .58, .60, .62])
        _, samples, _ = b.step(pwm, .03)
        lag_samples = list(samples)
        b.rpc.call("simContinueForTime", .03)
        _, samples = b.latest()
        lag_samples += samples
        observed = [r["control_signal_input"] for r in samples[-1]["rotors"]]
        truncated = b.rpc.call("getDynamicsStateBatch", lag_samples[0]["sequence"] - 1,
                               2, b.vehicle)
        evidence["tests"]["persistence_and_batch_limit"] = {
            "passed": bool(np.allclose(observed, pwm, atol=2e-6)
                           and len(truncated["samples"]) == 2),
            "observed_pwm": observed,
            "batch_limit_sequences": [s["sequence"] for s in truncated["samples"]]}
        evidence["all_physics_samples"]["motor_lag"] = lag_samples
        if check_overflow:
            b.rpc.call("armDisarm", False, b.vehicle)
            before_sequence = b.last_sequence
            b.rpc.timeout = 10
            deadline = time.monotonic() + 60
            while True:
                b.rpc.call("simContinueForTime", 3.)
                batch = b.rpc.call("getDynamicsStateBatch", before_sequence, 1, b.vehicle)
                if batch["buffer_last_sequence"] - before_sequence > 10000:
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError("Physics ring did not reach 10001 new samples within 60 wall seconds")
            evidence["tests"]["ring_overflow"] = {
                "passed": bool(batch["samples_dropped"]),
                "after_sequence": before_sequence,
                "buffer_first_sequence": batch["buffer_first_sequence"],
                "buffer_last_sequence": batch["buffer_last_sequence"],
                "samples_dropped": batch["samples_dropped"]}
        evidence["passed"] = all(r["passed"] for r in evidence["tests"].values())
    finally:
        b.stop()
        b.close()
    return evidence


def encode(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--local-config", type=Path, required=True)
    parser.add_argument("--exclusive-simulator", action="store_true", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--check-overflow", action="store_true")
    args = parser.parse_args()
    result = smoke(json.loads(args.local_config.read_text()), args.check_overflow)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, default=encode, indent=2) + "\n")
    print({k: v["passed"] for k, v in result["tests"].items()})
    raise SystemExit(0 if result["passed"] else 1)
