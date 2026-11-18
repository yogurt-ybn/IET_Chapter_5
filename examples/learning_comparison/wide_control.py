"""Flight interfaces shared by training and AirSim deployment."""

import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import block_diag, solve
from scipy.spatial.transform import Rotation
from iet_ch5.controllers.common import desired_attitude
from iet_ch5.controllers.lqr.controller import state_vector
from iet_ch5.allocation import Allocator
from iet_ch5.types import WrenchCommand
from controllers import policy_matrix
from wide_model import step

DT = 0.012


def reference_grid(source, duration, params):
    ts = np.arange(0, duration + 1.3, DT)
    refs = [source(t) for t in ts]
    force = np.array([r.a + [0.0, 0.0, params.gravity] for r in refs]) * params.mass
    rotations = Rotation.from_matrix(
        [desired_attitude(f, r.heading)[0] for f, r in zip(force, refs)]
    )
    rates = (rotations[:-1].inv() * rotations[1:]).as_rotvec() / DT
    rates = np.vstack((rates[0], 0.5 * (rates[:-1] + rates[1:]), rates[-1]))
    torque = np.gradient(rates, DT, axis=0) @ params.J.T + np.cross(rates, rates @ params.J.T)
    feed = np.column_stack((np.linalg.norm(force, axis=1), torque))
    desired = np.column_stack(
        ([r.p for r in refs], [r.v for r in refs], rotations.as_euler("xyz"), rates)
    )
    return desired, feed


def observation(states, target):
    error = states[..., :12] - target
    yaw = target[8]
    c, s = np.cos(yaw), np.sin(yaw)
    for i in (0, 3):
        a, b = error[..., i].copy(), error[..., i + 1].copy()
        error[..., i] = c * a + s * b
        error[..., i + 1] = -s * a + c * b
    return error


class ReferenceController:
    def __init__(self, config):
        self.config = config

    def reset(self, state, ref, params):
        self.params = params
        self.allocator = Allocator(params)
        self.hover = np.array([params.mass * params.gravity, 0.0, 0.0, 0.0])
        self.applied = self.hover

    def prepare_reference(self, source, duration):
        self.ref, self.feed = reference_grid(source, duration, self.params)

    def set_reference_source(self, source, t):
        self.index = int(round(t / DT))

    def set_applied_wrench(self, wrench):
        self.applied = wrench


class WidePolicy(ReferenceController):
    def reset(self, state, ref, params):
        super().reset(state, ref, params)
        self.K = policy_matrix(np.load(self.config["policy"])["theta"])

    def update(self, state, ref, dt):
        i = self.index
        u = self.K @ observation(state_vector(state), self.ref[i]) + self.feed[i]
        return WrenchCommand(u[0], u[1:]), {}


class WideMPC(ReferenceController):
    """Reference-linearised MPC. Recompute the constrained optimum on each state update."""

    def prepare_reference(self, source, duration):
        super().prepare_reference(source, duration)
        with np.load(self.config["model"]) as data:
            model = dict(data)
        self.targets = np.column_stack((self.ref, self.feed - self.hover))
        us = self.feed - self.hover
        x = self.targets

        def predict(states, actions):
            if "coef" in model:
                return step(states, actions, model)
            return states @ model["A"].T + actions @ model["B"].T + model["bias"]

        val = predict(x, us)
        # Batch central differences along the frozen reference, including actuators.
        dx = np.eye(16) * 1e-5
        du = np.eye(4) * 1e-5
        a = np.stack([(predict(x + d, us) - predict(x - d, us)) / 2e-5 for d in dx], axis=2)
        b = np.stack([(predict(x, us + d) - predict(x, us - d)) / 2e-5 for d in du], axis=2)
        c = val - np.einsum("nij,nj->ni", a, x) - np.einsum("nij,nj->ni", b, us)
        # Six successive 12 ms linearizations per constant-action block.
        n = len(x) - 6
        F = np.broadcast_to(np.eye(16), (n, 16, 16)).copy()
        G = np.zeros((n, 16, 4))
        offset = np.zeros((n, 16))
        for k in range(6):
            ak = a[k : k + n]
            F = ak @ F
            G = ak @ G + b[k : k + n]
            offset = np.einsum("nij,nj->ni", ak, offset) + c[k : k + n]
        self.blocks = F, G, offset
        self.W = np.tile([50, 50, 70, 5, 5, 8, 3, 3, 5, 0.08, 0.08, 0.15, 0, 0, 0, 0], 12)
        self.U = np.tile([0.06, 9, 9, 12], 12)
        self.mixer = block_diag(*([self.allocator.inverse] * 12))
        self.lower = -np.tile(self.allocator.inverse @ self.hover, 12)

    def update(self, state, ref, dt):
        i = self.index
        pos = np.r_[state_vector(state), self.applied - self.hover]
        moves = np.zeros((16, 48))
        designs = []
        errors = []
        F, G, c = self.blocks
        for j in range(12):
            k = i + 6 * j
            pos = F[k] @ pos + c[k]
            moves = F[k] @ moves
            moves[:, 4 * j : 4 * j + 4] += G[k]
            designs.append(moves.copy())
            errors.append(pos - self.targets[k + 6])
        design = np.vstack(designs)
        grad = design.T * self.W
        H = grad @ design + np.diag(self.U)
        aim = (self.feed[i + 3 : i + 72 : 6] - self.hover).ravel()
        g = grad @ np.concatenate(errors) - self.U * aim
        sol = solve(H, -g, assume_a="pos")
        rotor = self.mixer @ sol
        if np.any((rotor < self.lower) | (rotor > self.lower + 1)):
            qp = osqp.OSQP()
            qp.setup(
                P=sparse.csc_matrix(np.triu(H)),
                q=g,
                A=sparse.csc_matrix(self.mixer),
                l=self.lower,
                u=self.lower + 1,
                verbose=False,
                eps_abs=1e-6,
                eps_rel=1e-6,
            )
            res = qp.solve()
            if res.info.status_val not in (1, 2):
                raise RuntimeError(res.info.status)
            sol = res.x
        u = sol[:4] + self.hover
        return WrenchCommand(u[0], u[1:]), {}
