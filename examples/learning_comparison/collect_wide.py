"""Run predefined excitation flights with an SE(3) tracking controller."""

import sys, json
from pathlib import Path
import numpy as np
from excitation import flight
from iet_ch5.controllers.geometric.controller import Geometric
from iet_ch5.controllers.configuration import load_controller_configs
from iet_ch5.types import WrenchCommand
from split_routes import ROOT, TRAIN, VALID


class ExcitedGeometric(Geometric):
    def reset(self, state, ref, params):
        super().reset(state, ref, params)
        self.rng = np.random.default_rng(self.c["seed"])
        self.step = 0

    def update(self, state, ref, dt):
        u, info = super().update(state, ref, dt)
        if self.step % 8 == 0:
            self.noise = self.rng.uniform(-1, 1, 4) * [0.8, 0.018, 0.018, 0.008]
        self.step += 1
        value = u.vector() + self.noise
        return WrenchCommand(value[0], value[1:]), info


if __name__ == "__main__":
    env = json.loads(Path(sys.argv[1]).read_text())
    for i, name in enumerate((*TRAIN, VALID)):
        res = flight(
            "excitation",
            ExcitedGeometric,
            dict(load_controller_configs()["geometric"], seed=90 + i),
            str(ROOT / (name + ".json")),
            Path(sys.argv[2]) / name,
            env,
        )
        print(name, res.status, res.failure, flush=True)
