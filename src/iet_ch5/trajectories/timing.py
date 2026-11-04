"""Analytic C4 timing laws, kept identical to the verified legacy implementations."""

import numpy as np
from numpy.polynomial import Polynomial

# Degree-nine smoothstep, derivatives 1..4 zero at both endpoints.
SMOOTH = Polynomial([0, 0, 0, 0, 0, 126, -420, 540, -315, 70])
SMOOTH_INTEGRAL = SMOOTH.integ()


def blend(t, onset, duration):
    if t <= onset:
        return [0.] * 5
    if t >= onset + duration:
        return [1., 0., 0., 0., 0.]
    u = (t - onset) / duration
    return [SMOOTH.deriv(i)(u) / duration**i for i in range(5)]


def phase_harmonic(phase, frequency):
    """Time derivatives 0..4 of exp(i * frequency * phase(t))."""
    d = frequency * np.asarray(phase)
    z = np.exp(1j * d[0])
    return [z, 1j * d[1] * z, (1j * d[2] - d[1]**2) * z,
            (1j * d[3] - 3 * d[1] * d[2] - 1j * d[1]**3) * z,
            (1j * d[4] - 4 * d[1] * d[3] - 3 * d[2]**2
             - 6j * d[1]**2 * d[2] + d[1]**4) * z]


def cruise_phase(t, onset, duration, ramp):
    """One turn: smooth angular-speed ramps and a sustained constant-rate middle.

    Returns angle and its first four TIME derivatives. The integral of the speed
    smoothstep gives half a ramp's area, so omega = 2*pi/(duration-ramp).
    Constant angular rate is not constant Cartesian speed on a figure eight.
    """
    if not 0 < 2 * ramp < duration:
        raise ValueError('Cruise timing requires 0 < 2*ramp < duration')
    elapsed = t - onset
    if elapsed <= 0:
        return np.zeros(5)
    if elapsed >= duration:
        return np.array([2*np.pi, 0., 0., 0., 0.])
    omega = 2*np.pi / (duration - ramp)
    if ramp <= elapsed <= duration - ramp:
        return np.array([omega*(elapsed-ramp/2), omega, 0., 0., 0.])
    remaining = duration - elapsed
    u = min(elapsed, remaining) / ramp
    result = np.array([omega*ramp*SMOOTH_INTEGRAL(u), omega*SMOOTH(u),
                       omega/ramp*SMOOTH.deriv(1)(u),
                       omega/ramp**2*SMOOTH.deriv(2)(u),
                       omega/ramp**3*SMOOTH.deriv(3)(u)])
    if elapsed > duration - ramp:
        result[0] = 2*np.pi - result[0]
        result[[2, 4]] *= -1
    return result

