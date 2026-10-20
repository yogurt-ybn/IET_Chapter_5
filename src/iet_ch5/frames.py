"""Proper basis rotations: NED -> NWU and FRD -> FLU, both diag(1,-1,-1)."""

import numpy as np
from scipy.spatial.transform import Rotation

from .types import State, VehicleParameters

C = np.diag([1.0, -1.0, -1.0])


def vector(value):
    return np.array([value[k + "_val"] for k in "xyz"], dtype=float)


def rotation(quaternion):
    """AirSim map w/x/y/z -> scipy's explicitly ordered x/y/z/w quaternion."""
    return Rotation.from_quat([quaternion[k + "_val"] for k in "xyzw"]).as_matrix()


def state_from_sample(sample) -> State:
    k = sample["kinematics"]
    return State(C @ vector(k["position"]), C @ vector(k["linear_velocity"]),
                 C @ rotation(k["orientation"]) @ C,
                 C @ vector(k["angular_velocity"]), int(sample["timestamp"]))


def parameters_from_rpc(raw) -> VehicleParameters:
    """Require measured server parameters; there is deliberately no generic fallback."""
    poses = raw["rotor_poses"]
    p = VehicleParameters(
        mass=raw["mass"], J=C @ np.array(raw["inertia"]).reshape(3, 3) @ C,
        gravity=float(np.linalg.norm(vector(raw["gravity"]))),
        rotor_positions=np.array([C @ vector(r["position"]) for r in poses]),
        rotor_normals=np.array([C @ vector(r["normal"]) for r in poses]),
        rotor_directions=np.array([r["turning_direction"] for r in poses]),
        max_thrust=raw["max_thrust"], max_torque=raw["max_torque"],
        # EarthUtils::getAirDensity(0) = 101325/(287.053*288.15), NOT rounded 1.225.
        density_ratio=raw["air_density"] / (101325.0 / (287.053 * 288.15)),
        motor_time_constant=raw["control_signal_filter_tc"],
        physics_dt=raw["physics_loop_period_nanos"] * 1e-9)
    if (p.mass <= 0 or p.max_thrust <= 0 or p.gravity <= 0 or p.density_ratio <= 0
            or len(poses) != 4 or np.min(np.linalg.eigvalsh(p.J)) <= 0
            or not np.allclose(p.rotor_normals, [0, 0, 1])):
        raise ValueError("Unsupported or invalid active vehicle parameters")
    return p


def hat(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def vee(m):
    return np.array([m[2, 1], m[0, 2], m[1, 0]])
