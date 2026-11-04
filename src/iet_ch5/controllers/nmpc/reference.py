"""Original differential-flatness feedforward from the shared C4 reference information."""
import numpy as np

from ..common import unit_derivatives
from ...frames import hat, vee
from .model import quaternion_from_rotation


def flat_reference(reference, parameters):
    r, p = reference, parameters
    force = p.mass*(r.a+np.array([0., 0., p.gravity]))
    z, dz, ddz = unit_derivatives(force, p.mass*r.jerk, p.mass*r.snap)
    h = np.array([np.cos(r.heading), np.sin(r.heading), 0.])
    dh = r.heading_rate*np.array([-np.sin(r.heading), np.cos(r.heading), 0.])
    ddh = r.heading_acceleration*np.array([-np.sin(r.heading), np.cos(r.heading), 0.])-r.heading_rate**2*h
    y, dy, ddy = unit_derivatives(np.cross(z, h), np.cross(dz, h)+np.cross(z, dh),
                                  np.cross(ddz, h)+2*np.cross(dz, dh)+np.cross(z, ddh))
    x = np.cross(y, z)
    dx, ddx = np.cross(dy, z)+np.cross(y, dz), np.cross(ddy, z)+2*np.cross(dy, dz)+np.cross(y, ddz)
    R, dR, ddR = np.column_stack([x, y, z]), np.column_stack([dx, dy, dz]), np.column_stack([ddx, ddy, ddz])
    omega = vee(R.T@dR)
    domega = vee(R.T@ddR-hat(omega)@hat(omega))
    wrench = np.r_[np.linalg.norm(force), p.J@domega+np.cross(omega, p.J@omega)]
    return np.r_[r.p, r.v, quaternion_from_rotation(R), omega, wrench]


class ReferenceTable:
    """Offline precomputation before evaluated control; online interpolation is timed."""
    def __init__(self, function, duration, parameters, period):
        self.times = np.arange(0., duration+period, period)
        self.values = np.array([flat_reference(function(float(t)), parameters) for t in self.times])
        for i in range(1, len(self.values)):
            if self.values[i, 6:10]@self.values[i-1, 6:10] < 0:
                self.values[i, 6:10] *= -1

    def at(self, times):
        values = np.column_stack([np.interp(times, self.times, self.values[:, i]) for i in range(17)])
        values[:, 6:10] /= np.linalg.norm(values[:, 6:10], axis=1)[:, None]
        return values
