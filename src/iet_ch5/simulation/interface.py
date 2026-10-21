"""Shared simulation contract. Controllers consume only State/Reference/WrenchCommand.

Physics dictionaries are archival v1 records for independent evaluation, never controller
inputs. Their decoded States always use SI NWU/FLU. Motor ordering and actuator mapping
belong to the measured VehicleParameters and the shared Allocator.
"""

from dataclasses import dataclass
from typing import Protocol, runtime_checkable
import numpy as np

from ..types import State, VehicleParameters


@dataclass
class SceneImage:
    png: bytes
    timestamp_ns: int
    width: int
    height: int


@runtime_checkable
class SimulationBackend(Protocol):
    parameters: VehicleParameters
    last_timing: dict
    provenance: dict

    def initialize(self, position) -> None: ...
    def set_initial_kinematics(self, position) -> None: ...
    def paused_truth(self, timestamp_ns: int) -> State: ...
    def step(self, pwm: np.ndarray, duration: float) -> tuple[State, list[dict], float]: ...
    def decode_physics_sample(self, sample: dict) -> State: ...
    def set_wind(self, velocity_nwu) -> None: ...
    def configure_camera(self, camera: str, position_nwu, target_nwu) -> dict: ...
    def camera_info(self, camera: str, external=True) -> dict: ...
    def draw_paths(self, reference_nwu, actual_nwu) -> None: ...
    def capture_scene(self, camera='0', external=False) -> SceneImage: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...
