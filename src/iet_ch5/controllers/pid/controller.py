"""Original cascaded position/velocity, attitude/rate controller with bounded integrators."""

import numpy as np

from ..common import attitude_error, desired_attitude
from ...types import WrenchCommand


class PID:
    def __init__(self, config):
        self.c = config

    def reset(self, initial_state, reference, parameters):
        self.p = parameters
        self.velocity_integral = np.zeros(3)
        self.rate_integral = np.zeros(3)
        self.filtered_rate = initial_state.omega.copy()
        self.force = np.array([0., 0., parameters.mass * parameters.gravity])
        self.Rd = np.eye(3)
        self.outer_elapsed = 1e3
        self.allocation_saturated = False

    def observe_applied(self, estimated_wrench, allocation_residual, dt):
        # Conditional integration uses the same allocator for all controllers.
        self.allocation_saturated = np.linalg.norm(allocation_residual) > 1e-6

    def update(self, state, reference, dt):
        c = self.c
        self.outer_elapsed += dt
        if self.outer_elapsed >= c["outer_period"]:
            h = min(self.outer_elapsed, c["outer_period"] * 2)
            self.outer_elapsed = 0.
            vd = reference.v + np.array(c["position_kp"]) * (reference.p - state.p)
            vd = np.clip(vd, -c["velocity_limit"], c["velocity_limit"])
            ev = vd - state.v
            if not self.allocation_saturated:
                self.velocity_integral += ev * h
                self.velocity_integral = np.clip(self.velocity_integral,
                                                -c["velocity_integral_limit"],
                                                c["velocity_integral_limit"])
            raw = (reference.a + np.array(c["velocity_kp"]) * ev
                   + np.array(c["velocity_ki"]) * self.velocity_integral)
            acc = np.clip(raw, -c["acceleration_limit"], c["acceleration_limit"])
            # Back calculation also unwinds the velocity integrator when acceleration is limited.
            ki = np.array(c["velocity_ki"])
            self.velocity_integral += h * (acc - raw) / np.maximum(ki, .01)
            self.velocity_integral = np.clip(self.velocity_integral,
                                            -c["velocity_integral_limit"],
                                            c["velocity_integral_limit"])
            self.force = self.p.mass * (acc + np.array([0., 0., self.p.gravity]))
            self.Rd, _ = desired_attitude(self.force, reference.heading, self.Rd)
        eR = attitude_error(state.R, self.Rd)
        rate_target = -np.array(c["attitude_kp"]) * eR
        rate_target += state.R.T @ self.Rd @ np.array([0., 0., reference.heading_rate])
        rate_target = np.clip(rate_target, -c["rate_limit"], c["rate_limit"])
        alpha = 1 - np.exp(-dt / c["rate_filter_tc"])
        self.filtered_rate += alpha * (state.omega - self.filtered_rate)
        ew = rate_target - self.filtered_rate
        if not self.allocation_saturated:
            self.rate_integral = np.clip(self.rate_integral + ew * dt,
                                         -c["rate_integral_limit"], c["rate_integral_limit"])
        angular_acc = np.array(c["rate_kp"]) * ew + np.array(c["rate_ki"]) * self.rate_integral
        tau = self.p.J @ angular_acc + np.cross(state.omega, self.p.J @ state.omega)
        return WrenchCommand(max(0., self.force @ state.R[:, 2]), tau), {
            "attitude_error": eR, "rate_error": ew, "desired_R": self.Rd,
            "rate_target_body": rate_target,
            "velocity_integral": self.velocity_integral.copy(),
            "rate_integral": self.rate_integral.copy(), "outer_period": c["outer_period"]}
