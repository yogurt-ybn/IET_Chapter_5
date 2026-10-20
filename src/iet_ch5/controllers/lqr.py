"""Discrete near-hover 12-state full-wrench LQR, derived for NWU/FLU."""

import numpy as np
from scipy.linalg import expm, solve_discrete_are
from scipy.spatial.transform import Rotation

from ..types import WrenchCommand


def hover_model(p, dt):
    # State [p(3),v(3),small FLU roll/pitch/yaw(3),omega(3)].
    # delta vx_dot = g*pitch, delta vy_dot = -g*roll; heading equilibrium is zero.
    A = np.zeros((12, 12))
    B = np.zeros((12, 4))
    A[:3, 3:6] = np.eye(3)
    A[3, 7] = p.gravity
    A[4, 6] = -p.gravity
    A[6:9, 9:12] = np.eye(3)
    B[5, 0] = 1 / p.mass
    B[9:12, 1:4] = np.linalg.inv(p.J)
    augmented = np.zeros((16, 16))
    augmented[:12, :12] = A
    augmented[:12, 12:] = B
    discrete = expm(augmented * dt)
    return discrete[:12, :12], discrete[:12, 12:]


def state_vector(state):
    return np.r_[state.p, state.v, Rotation.from_matrix(state.R).as_euler("xyz"), state.omega]


def reference_vector(ref, p):
    # Near-hover feedforward tilt for the desired inertial acceleration (fixed zero heading).
    return np.r_[ref.p, ref.v, -ref.a[1] / p.gravity, ref.a[0] / p.gravity, ref.heading,
                 -ref.jerk[1] / p.gravity, ref.jerk[0] / p.gravity, ref.heading_rate]


class LQR:
    def __init__(self, config):
        self.c = config

    def reset(self, initial_state, reference, parameters):
        self.p = parameters
        self.A, self.B = hover_model(parameters, self.c["design_dt"])
        self.Q = np.diag(self.c["Q"])
        self.W = np.diag(self.c["W"])
        self.P = solve_discrete_are(self.A, self.B, self.Q, self.W)
        self.K = np.linalg.solve(self.W + self.B.T @ self.P @ self.B,
                                 self.B.T @ self.P @ self.A)

    def update(self, state, reference, dt):
        deviation = state_vector(state) - reference_vector(reference, self.p)
        deviation[8] = np.arctan2(np.sin(deviation[8]), np.cos(deviation[8]))
        feedforward = np.r_[self.p.mass * (self.p.gravity + reference.a[2]),
                            self.p.J @ np.array([-reference.snap[1] / self.p.gravity,
                                                reference.snap[0] / self.p.gravity,
                                                reference.heading_acceleration])]
        command = feedforward - self.K @ deviation
        return WrenchCommand(command[0], command[1:]), {
            "state_deviation": deviation, "design_dt": self.c["design_dt"],
            "measured_dt": dt}
