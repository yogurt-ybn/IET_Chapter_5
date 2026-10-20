"""C4 transfers, circles and figure eights, with analytic derivatives through snap."""

import numpy as np
from numpy.polynomial import Polynomial

from ..types import Reference

# Degree-nine smoothstep, derivatives 1..4 zero at both endpoints.
SMOOTH = Polynomial([0, 0, 0, 0, 0, 126, -420, 540, -315, 70])


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


class Task:
    def __init__(self, name, config):
        self.name = name
        self.c = config

    @property
    def tracking_interval(self):
        if self.name in ('circle', 'figure_eight'):
            return (self.c[self.name + '_onset'],
                    self.c[self.name + '_onset'] + self.c[self.name + '_duration'])
        return None

    @property
    def duration(self):
        if self.tracking_interval is not None:
            return self.c[self.name + '_total_duration']
        return self.c['duration']

    def at(self, t):
        c = self.c
        origin = np.array(c["initial_position"], dtype=float)
        if self.name in ("hover_transfer", "disturbance", "mismatch", "noise_delay"):
            derivatives = blend(t, c["transfer_onset"], c["transfer_duration"])
            delta = np.array(c["transfer_displacement"])
            values = [delta * value for value in derivatives]
            values[0] += origin
            return Reference(*values)
        if self.name == "circle":
            # Progress ramp: theta itself has zero first four derivatives at entry/exit.
            # Complete one loop, beginning/ending at initial_position without a position jump.
            d = np.array(blend(t, c["circle_onset"], c["circle_duration"])) * 2 * np.pi
            z = np.exp(1j * d[0])
            dz = 1j * d[1] * z
            ddz = (1j * d[2] - d[1]**2) * z
            dddz = (1j * d[3] - 3 * d[1] * d[2] - 1j * d[1]**3) * z
            ddddz = (1j * d[4] - 4 * d[1] * d[3] - 3 * d[2]**2
                     - 6j * d[1]**2 * d[2] + d[1]**4) * z
            values = [c["circle_radius"] * np.array([x.real, x.imag, 0.])
                      for x in [z - 1, dz, ddz, dddz, ddddz]]
            values[0] += origin
            return Reference(*values)
        if self.name == 'figure_eight':
            # Gerono figure eight: one cosine and a double-frequency sine.
            # Start/end at the right-hand tip, with two distinct centre crossings.
            phase = np.array(blend(t, c['figure_eight_onset'],
                                   c['figure_eight_duration'])) * 2 * np.pi
            first, second = phase_harmonic(phase, 1), phase_harmonic(phase, 2)
            values = [np.array([c['figure_eight_half_width_m'] * x.real,
                                c['figure_eight_half_height_m'] * y.imag, 0.])
                      for x, y in zip(first, second)]
            values[0] += origin - np.array([c['figure_eight_half_width_m'], 0., 0.])
            return Reference(*values)
        raise ValueError(f"Unknown task {self.name}")
