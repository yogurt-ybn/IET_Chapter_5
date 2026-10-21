"""Standard coupled-yaw SE(3), upward thrust; FDCL derivative construction (MIT).

Moment law follows Lee/Leok/McClamroch 2010, not FDCL's decoupled-yaw variant.
Desired-minus-actual translation errors; actual-minus-desired SO(3) errors.
"""

import numpy as np

from ..common import attitude_error, desired_attitude, unit_derivatives
from ...frames import hat, vee
from ...types import WrenchCommand


class Geometric:
    def __init__(self, config):
        self.c = config

    def reset(self, initial_state, reference, parameters):
        self.p = parameters
        self.last_Rd = np.eye(3)
        self.force_compensation = np.zeros(3)

    def update(self, state, reference, dt):
        p, c = self.p, self.c
        kx, kv = np.array(c["position_kp"]), np.array(c["velocity_kp"])
        force = (p.mass * (reference.a + np.array([0., 0., p.gravity]))
                 + kx * (reference.p - state.p) + kv * (reference.v - state.v)
                 - self.force_compensation)
        b3 = state.R[:, 2]
        b3dot = state.R @ hat(state.omega)[:, 2]
        f = max(0., force @ b3)
        # Nominal dynamics provide derivatives. The observer's slow force estimate is held
        # constant over this derivative calculation, an explicit approximation in DOB mode.
        acceleration = f / p.mass * b3 - np.array([0., 0., p.gravity])
        dforce = p.mass * reference.jerk + kx * (reference.v - state.v)
        dforce += kv * (reference.a - acceleration)
        fdot = dforce @ b3 + force @ b3dot
        jerk = fdot / p.mass * b3 + f / p.mass * b3dot
        ddforce = p.mass * reference.snap + kx * (reference.a - acceleration)
        ddforce += kv * (reference.jerk - jerk)
        singular = False
        try:
            z, dz, ddz = unit_derivatives(force, dforce, ddforce)
            psi, dpsi, ddpsi = reference.heading, reference.heading_rate, reference.heading_acceleration
            h = np.array([np.cos(psi), np.sin(psi), 0.])
            dh = dpsi * np.array([-np.sin(psi), np.cos(psi), 0.])
            ddh = ddpsi * np.array([-np.sin(psi), np.cos(psi), 0.]) - dpsi**2 * h
            y, dy, ddy = unit_derivatives(np.cross(z, h), np.cross(dz, h) + np.cross(z, dh),
                np.cross(ddz, h) + 2 * np.cross(dz, dh) + np.cross(z, ddh))
            x = np.cross(y, z)
            dx = np.cross(dy, z) + np.cross(y, dz)
            ddx = np.cross(ddy, z) + 2 * np.cross(dy, dz) + np.cross(y, ddz)
            Rd = np.column_stack([x, y, z])
            dRd = np.column_stack([dx, dy, dz])
            ddRd = np.column_stack([ddx, ddy, ddz])
            wd = vee(Rd.T @ dRd)
            dwd = vee(Rd.T @ ddRd - hat(wd) @ hat(wd))
        except ValueError:
            Rd, _ = desired_attitude(force, reference.heading, self.last_Rd)
            wd, dwd = np.zeros(3), np.zeros(3)
            singular = True
        self.last_Rd = Rd
        eR = attitude_error(state.R, Rd)
        transform = state.R.T @ Rd
        ew = state.omega - transform @ wd
        tau = (-np.array(c["attitude_kp"]) * eR - np.array(c["rate_kp"]) * ew
               + np.cross(state.omega, p.J @ state.omega)
               - p.J @ (hat(state.omega) @ transform @ wd - transform @ dwd))
        return WrenchCommand(f, tau), {"desired_R": Rd, "desired_rate": wd,
            "attitude_error": eR, "rate_error": ew, "direction_singularity": singular,
            "desired_force_nwu": force}
