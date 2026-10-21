#!/usr/bin/env python3
"""Verify wind response, actuator saturation and solver failure on the isolated simulator."""

import argparse
import json
from pathlib import Path

import numpy as np

from iet_ch5.allocation import Allocator
from iet_ch5.backends.airsim_backend import AirSimBackend
from iet_ch5.controllers import make_controller
from iet_ch5.logging import dump, revision
from iet_ch5.types import Reference, WrenchCommand


def verify(config):
    b = AirSimBackend(config["host"], config["port"], config["vehicle"], exclusive=True)
    allocator = Allocator(b.parameters)
    evidence = {"software": revision(), "tests": {},
                "force_pulse": {"status": "unavailable", "reason": "Active RPC server has no plant external-force/moment injection API; wind velocity is not a prescribed force."}}
    hover = allocator.allocate(WrenchCommand(b.parameters.mass*b.parameters.gravity, np.zeros(3)))
    try:
        for name, wind in [("zero_wind", [0.,0.,0.]), ("wind", [8.,-4.,0.])]:
            b.initialize()
            rows = []
            for _ in range(15):
                state, samples, _ = b.step(hover.pwm, .012)
                rows += samples
            start = state
            b.set_wind(wind)
            for _ in range(15):
                state, samples, _ = b.step(hover.pwm, .012)
                rows += samples
            delta = state.v-start.v
            evidence["tests"][name] = {"velocity_nwu_m_s": wind, "delta_vehicle_v_nwu_m_s": delta,
                "passed": bool(np.linalg.norm(delta[:2]) < .001 if name=="zero_wind" else delta[0]>.01 and delta[1]<-.001),
                "physics_samples": rows, "motor_pwm": hover.pwm}
        b.initialize()
        saturation = allocator.allocate(WrenchCommand(100., np.zeros(3)))
        _, samples, _ = b.step(saturation.pwm, .03)
        evidence["tests"]["saturation"] = {"passed": bool(saturation.saturated.all()),
            "requested_wrench": [100.,0.,0.,0.], "allocated_pwm": saturation.pwm,
            "residual": saturation.residual, "physics_samples": samples,
            "note": "Actuator diagnostic, not external-force injection"}
        b.initialize()
        state, samples, _ = b.step(hover.pwm, .012)
        cfg = json.loads(Path("configs/controllers.json").read_text())["mpc"]
        cfg["solver_time_limit_s"] = 1e-9
        controller = make_controller("mpc", cfg)
        ref = Reference(np.array([.5,.3,2.3]))
        controller.reset(state, ref, b.parameters)
        command, diag = controller.update(state, ref, .036)
        actual = allocator.allocate(command)
        _, samples, _ = b.step(actual.pwm, .036)
        evidence["tests"]["solver_timeout_fallback"] = {"passed": bool(diag["fallback"]),
            "diagnostics": diag, "requested_wrench": command.vector(),
            "allocated_pwm": actual.pwm, "physics_samples": samples}
        evidence["passed"] = all(x["passed"] for x in evidence["tests"].values())
    finally:
        b.stop()
        b.close()
    return evidence


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--local-config", type=Path, required=True)
    p.add_argument("--exclusive-simulator", action="store_true", required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    result = verify(json.loads(args.local_config.read_text()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(dump(result)+"\n")
    print({k:v["passed"] for k,v in result["tests"].items()})
    raise SystemExit(0 if result["passed"] else 1)
