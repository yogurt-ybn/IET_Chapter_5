"""Receding-horizon state tracking and a separately reward-trained direct wrench policy."""

import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import block_diag, cho_factor, cho_solve
from iet_ch5.allocation import Allocator
from iet_ch5.controllers.lqr.controller import state_vector, reference_vector
from iet_ch5.types import WrenchCommand


class IdentifiedMPC:
    def __init__(self, config):
        self.model = np.load(config["model"])

    def reset(self, state, ref, params):
        self.params = params
        A, B, bias, self.hover = [self.model[k] for k in ("A", "B", "bias", "hover")]
        # Twelve constant-wrench blocks, each six control ticks; resolve every tick.
        count, ticks = 12, 6
        self.preview = (1 + np.arange(count)) * ticks * 0.012
        lift = sum(np.linalg.matrix_power(A, i) for i in range(ticks))
        F, G, offset = np.linalg.matrix_power(A, ticks), lift @ B, lift @ bias
        transition, moves, drift = np.eye(16), np.zeros((16, 4 * count)), np.zeros(16)
        ss, uu, cc = [], [], []
        for i in range(count):
            transition, moves, drift = F @ transition, F @ moves, F @ drift + offset
            moves[:, 4 * i : 4 * i + 4] += G
            ss.append(transition.copy())
            uu.append(moves.copy())
            cc.append(drift.copy())
        self.states, controls, self.offset = np.vstack(ss), np.vstack(uu), np.concatenate(cc)
        weights = np.tile([50, 50, 70, 5, 5, 8, 3, 3, 5, 0.08, 0.08, 0.15, 0, 0, 0, 0], count)
        self.gradient = controls.T * weights
        H = self.gradient @ controls + np.diag(np.tile([0.06, 9, 9, 12], count))
        self.factor = cho_factor(H)
        inv = Allocator(params).inverse
        self.mixer = block_diag(*([inv] * count))
        self.lower = -np.tile(inv @ self.hover, count)
        self.upper = 1 + self.lower
        self.qp = osqp.OSQP()
        self.qp.setup(
            P=sparse.csc_matrix(np.triu(H)),
            q=np.zeros(4 * count),
            A=sparse.csc_matrix(self.mixer),
            l=self.lower,
            u=self.upper,
            verbose=False,
            eps_abs=1e-6,
            eps_rel=1e-6,
        )
        self.applied = self.hover.copy()

    def set_reference_source(self, source, t):
        self.source, self.t = source, t

    def prepare_reference(self, source, duration):
        self.times = np.arange(0, duration + 1.2, 0.012)
        self.refs = np.array(
            [np.r_[reference_vector(source(t), self.params), np.zeros(4)] for t in self.times]
        )

    def set_applied_wrench(self, applied):
        self.applied = applied

    def update(self, state, reference, dt):
        x = np.r_[state_vector(state), self.applied - self.hover]
        ids = np.rint((self.t + self.preview) / 0.012).astype(int).clip(0, len(self.times) - 1)
        gradient = self.gradient @ (self.states @ x + self.offset - self.refs[ids].ravel())
        moves = cho_solve(self.factor, -gradient)
        rotor = self.mixer @ moves
        bounded = np.any((rotor < self.lower) | (rotor > self.upper))
        if bounded:
            self.qp.update(q=gradient)
            sol = self.qp.solve()
            if sol.info.status_val not in (1, 2):
                raise RuntimeError(sol.info.status)
            moves = sol.x
        u = moves[:4] + self.hover
        return WrenchCommand(u[0], u[1:]), {"constrained_solve": bool(bounded)}


# Each row selects tracking errors for a single wrench channel; coefficients are
# trainable. Signs encode the chapter's NWU/FLU convention.
FEATURES = [[2, 5], [1, 4, 6, 9], [0, 3, 7, 10], [8, 11]]
START = np.array([-2.0, -2.0, 0.06, 0.08, -0.38, -0.09, -0.06, -0.08, -0.38, -0.09, -0.22, -0.09])


def policy_matrix(theta):
    matrix = np.zeros(theta.shape[:-1] + (4, 12))
    offset = 0
    for axis, columns in enumerate(FEATURES):
        for column in columns:
            matrix[..., axis, column] = START[offset] * np.exp(theta[..., offset])
            offset += 1
    return matrix


class DirectPolicy:
    def __init__(self, config):
        self.matrix = policy_matrix(np.load(config["policy"])["theta"])

    def reset(self, state, reference, params):
        self.params = params

    def update(self, state, reference, dt):
        error = state_vector(state) - reference_vector(reference, self.params)
        u = self.matrix @ error
        u[0] += self.params.mass * (self.params.gravity + reference.a[2])
        return WrenchCommand(u[0], u[1:]), {}
