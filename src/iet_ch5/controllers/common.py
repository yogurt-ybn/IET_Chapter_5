"""Shared attitude construction and rotation errors; no simulator commands."""

import numpy as np

from ..frames import vee


def desired_attitude(force, heading, previous=None):
    norm = np.linalg.norm(force)
    if norm < 1e-7:
        return (np.eye(3) if previous is None else previous.copy()), True
    z = force / norm
    h = np.array([np.cos(heading), np.sin(heading), 0.])
    y = np.cross(z, h)
    singular = np.linalg.norm(y) < 1e-5
    if singular:
        # Select the world axis least parallel to thrust, keeping a right-handed frame.
        h = np.eye(3)[np.argmin(np.abs(z))]
        y = np.cross(z, h)
    y /= np.linalg.norm(y)
    return np.column_stack([np.cross(y, z), y, z]), singular


def attitude_error(R, Rd):
    # Actual-minus-desired SO(3) error with explicitly negative feedback.
    return .5 * vee(Rd.T @ R - R.T @ Rd)


def unit_derivatives(x, dx, ddx):
    """FDCL deriv_unit_vector algebra, MIT; adapted to arrays and explicit degeneracy check."""
    n = np.linalg.norm(x)
    if n < 1e-7:
        raise ValueError("Degenerate direction has no differentiable normalization")
    c = x @ dx
    u = x / n
    du = dx / n - x * c / n**3
    ddu = (ddx / n - 2 * dx * c / n**3
           - x * (dx @ dx + x @ ddx) / n**3 + 3 * x * c**2 / n**5)
    return u, du, ddu
