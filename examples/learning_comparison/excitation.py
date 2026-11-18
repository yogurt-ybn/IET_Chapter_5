"""Collect independently excited state transitions through the chapter's motor bridge."""

import argparse
import json
from pathlib import Path
import numpy as np
from iet_ch5 import runner
from iet_ch5.controllers.lqr.controller import LQR
from iet_ch5.controllers.configuration import load_controller_configs
from iet_ch5.experiments.configuration import build_runtime
from iet_ch5.trajectories import load_trajectory
from iet_ch5.types import WrenchCommand


class Excitation(LQR):
    def reset(self, state, reference, parameters):
        super().reset(state, reference, parameters)
        self.rng = np.random.default_rng(self.c["seed"])
        self.tick = 0

    def update(self, state, reference, dt):
        base, info = super().update(state, reference, dt)
        if self.tick % 10 == 0:
            self.offset = self.rng.uniform(-1, 1, 4) * [1.6, 0.035, 0.035, 0.012]
        self.tick += 1
        u = base.vector() + self.offset
        return WrenchCommand(u[0], u[1:]), info


def flight(method, factory, config, trajectory, output, environment):
    runtime = build_runtime(
        load_trajectory(trajectory),
        {"kind": "nominal", "id": "nominal", "seeds": [0]},
        json.loads(Path("configs/experiments/runtime.json").read_text()),
    )
    return runner.run(
        environment,
        {method: config},
        runtime,
        method,
        trajectory,
        output,
        controller_factory=lambda name, cfg: factory(cfg),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("environment", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    for seed, trajectory in enumerate(["circle", "helix", "transfer"]):
        config = dict(load_controller_configs()["lqr"], seed=seed)
        result = flight(
            "identification",
            Excitation,
            config,
            trajectory,
            args.output / trajectory,
            json.loads(args.environment.read_text()),
        )
        print(
            trajectory,
            result.status,
            result.failure if hasattr(result, "failure") else "",
            flush=True,
        )
