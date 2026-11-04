"""Shared physical records. Arrays use SI units and NWU/FLU coordinates."""

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]


@dataclass
class State:
    p: Array
    v: Array
    R: Array
    omega: Array
    timestamp_ns: int
    provenance: str = "AirSim physics truth; body angular velocity"


@dataclass
class Reference:
    p: Array
    v: Array = field(default_factory=lambda: np.zeros(3))
    a: Array = field(default_factory=lambda: np.zeros(3))
    jerk: Array = field(default_factory=lambda: np.zeros(3))
    snap: Array = field(default_factory=lambda: np.zeros(3))
    heading: float = 0.0
    heading_rate: float = 0.0
    heading_acceleration: float = 0.0


@dataclass
class VehicleParameters:
    mass: float
    J: Array
    gravity: float
    rotor_positions: Array
    rotor_normals: Array
    rotor_directions: Array
    max_thrust: float
    max_torque: float
    density_ratio: float
    motor_time_constant: float
    physics_dt: float


@dataclass
class WrenchCommand:
    f: float
    tau: Array

    def vector(self) -> Array:
        return np.r_[self.f, self.tau]


@dataclass
class RunResult:
    run_id: str
    controller: str
    status: str
    log_path: str
    metrics: dict
    failure: str | None = None


class Controller(Protocol):
    def reset(self, initial_state: State, reference: Reference,
              parameters: VehicleParameters) -> None: ...

    def update(self, state: State, reference: Reference,
               dt: float) -> tuple[WrenchCommand, dict]: ...
