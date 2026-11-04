"""Bounded translational force observer around an identical fixed geometric controller.

Original constant-mass specialization of a low-pass disturbance residual (Chen et al.
2000, section II-B). Implicit-Euler momentum-increment realization avoids a standalone
unfiltered acceleration signal. This is force-only compensation, not a moment observer.
"""

import numpy as np

from ..geometric import Geometric


class DOB:
    def __init__(self, config):
        self.c = config
        self.nominal = Geometric(config["nominal"])

    def reset(self, initial_state, reference, parameters):
        self.p = parameters
        self.nominal.reset(initial_state, reference, parameters)
        self.previous_v = initial_state.v.copy()
        self.previous_b3 = initial_state.R[:, 2].copy()
        self.estimate = np.zeros(3)
        self.applied_collective = parameters.mass * parameters.gravity

    def set_applied_collective(self, interval_mean_thrust):
        self.applied_collective = interval_mean_thrust

    def update(self, state, reference, dt):
        gain = self.c["bandwidth_rad_s"]
        known_force = self.applied_collective * .5 * (self.previous_b3 + state.R[:, 2])
        known_force -= np.array([0., 0., self.p.mass * self.p.gravity])
        momentum_increment = self.p.mass * (state.v - self.previous_v)
        self.estimate = (self.estimate + gain * momentum_increment - gain * dt * known_force) / (1 + gain * dt)
        self.estimate = np.clip(self.estimate, -self.c["force_bound_N"], self.c["force_bound_N"])
        self.previous_v = state.v.copy()
        self.previous_b3 = state.R[:, 2].copy()
        self.nominal.force_compensation = self.estimate.copy() if self.c["enabled"] else np.zeros(3)
        command, diagnostic = self.nominal.update(state, reference, dt)
        diagnostic.update(force_estimate_nwu_N=self.estimate.copy(),
                          observer_enabled=self.c["enabled"],
                          observer_bandwidth_rad_s=gain,
                          applied_collective_estimate_N=self.applied_collective)
        return command, diagnostic
