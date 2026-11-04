"""One geometry-derived bounded allocator for every controller.

AirSim input is proportional to filtered thrust before air-density correction.
Rotor force and reaction moment both include the same environmental density ratio.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear

from .types import VehicleParameters, WrenchCommand


@dataclass
class Allocation:
    pwm: np.ndarray
    predicted_wrench: np.ndarray
    residual: np.ndarray
    saturated: np.ndarray


class Allocator:
    def __init__(self, p: VehicleParameters):
        thrust = p.rotor_normals * (p.max_thrust * p.density_ratio)
        moment = np.cross(p.rotor_positions, thrust)
        moment += (p.rotor_normals * p.rotor_directions[:, None]
                   * p.max_torque * p.density_ratio)
        self.B = np.vstack([thrust[:, 2], moment.T])
        if np.linalg.matrix_rank(self.B) != 4:
            raise ValueError("Singular rotor allocation")
        self.inverse = np.linalg.inv(self.B)
        # Normalize physical channels; moments have different units from collective thrust.
        self.scale = np.array([1 / (p.mass * p.gravity), 1 / 0.3, 1 / 0.3, 1 / 0.1])

    def allocate(self, command: WrenchCommand) -> Allocation:
        requested = command.vector()
        if not np.isfinite(requested).all():
            raise ValueError("Nonfinite wrench command")
        pwm = self.inverse @ requested
        if np.any(pwm < 0) or np.any(pwm > 1):
            pwm = lsq_linear(self.scale[:, None] * self.B, self.scale * requested,
                             bounds=(0, 1), tol=1e-10).x
        achieved = self.B @ pwm
        return Allocation(pwm, achieved, requested - achieved,
                          (pwm <= 1e-6) | (pwm >= 1 - 1e-6))

    def from_telemetry(self, rotors):
        """Model estimate of applied wrench from measured filtered motor state.

        Rotor telemetry omits the final density factor and is published after pose integration.
        This is an estimate with documented phase, not a direct load-cell measurement.
        """
        return self.B @ np.array([r["control_signal_filtered"] for r in rotors])
