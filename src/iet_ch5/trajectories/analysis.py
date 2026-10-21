"""Reference-only physical screen, not a closed-loop feasibility/stability guarantee."""

import numpy as np
from ..allocation import Allocator
from ..controllers.common import unit_derivatives
from ..frames import hat, vee
from .library import Trajectory


def screen(config, parameters=None, samples=801):
    task = Trajectory(config)
    a, b = task.tracking_interval
    times = np.linspace(a, b, samples)
    refs = [task.at(float(t)) for t in times]
    v = np.array([r.v for r in refs])
    acc = np.array([r.a for r in refs])
    g = parameters.gravity if parameters is not None else 9.81
    tilt = np.arctan2(np.linalg.norm(acc[:, :2], axis=1), acc[:, 2]+g)
    result = {'trajectory': config['id'], 'tier': config['tier'], 'duration_s': task.duration,
              'traversal_s': b-a, 'screen_samples': samples,
              'reference_peak_speed_m_s': float(np.linalg.norm(v, axis=1).max()),
              'reference_peak_axis_speed_m_s': np.abs(v).max(axis=0).tolist(),
              'reference_peak_acceleration_m_s2': float(np.linalg.norm(acc, axis=1).max()),
              'reference_peak_tilt_rad': float(tilt.max()),
              'reference_peak_heading_rate_rad_s': float(max(abs(r.heading_rate) for r in refs)),
              'physical_parameters_available': parameters is not None,
              'scope': 'Reference feedforward static rotor coupling; no motor lag, closed-loop or obstacle guarantee'}
    if parameters is None:
        result['physical_feedforward_feasible'] = None
        return result
    allocator = Allocator(parameters)
    commands, rates, moments = [], [], []
    for r in refs:
        force = parameters.mass*(r.a + np.array([0., 0., g]))
        z, dz, ddz = unit_derivatives(force, parameters.mass*r.jerk, parameters.mass*r.snap)
        h = np.array([np.cos(r.heading), np.sin(r.heading), 0.])
        dh = r.heading_rate*np.array([-np.sin(r.heading), np.cos(r.heading), 0.])
        ddh = r.heading_acceleration*np.array([-np.sin(r.heading), np.cos(r.heading), 0.])-r.heading_rate**2*h
        y, dy, ddy = unit_derivatives(np.cross(z, h), np.cross(dz, h)+np.cross(z, dh),
                                      np.cross(ddz, h)+2*np.cross(dz, dh)+np.cross(z, ddh))
        x = np.cross(y, z)
        dx = np.cross(dy, z)+np.cross(y, dz)
        ddx = np.cross(ddy, z)+2*np.cross(dy, dz)+np.cross(y, ddz)
        R = np.column_stack([x, y, z])
        dR = np.column_stack([dx, dy, dz])
        ddR = np.column_stack([ddx, ddy, ddz])
        omega = vee(R.T@dR)
        domega = vee(R.T@ddR-hat(omega)@hat(omega))
        moment = parameters.J@domega+np.cross(omega, parameters.J@omega)
        commands.append(allocator.inverse@np.r_[np.linalg.norm(force), moment])
        rates.append(np.linalg.norm(omega))
        moments.append(moment)
    commands = np.array(commands)
    result.update(reference_peak_body_rate_rad_s=float(max(rates)),
                  reference_peak_abs_moment_Nm=np.abs(moments).max(axis=0).tolist(),
                  feedforward_rotor_input_min=float(commands.min()),
                  feedforward_rotor_input_max=float(commands.max()),
                  physical_feedforward_feasible=bool(commands.min() >= 0 and commands.max() <= 1
                      and tilt.max() < config['safety']['tilt_limit_rad']))
    return result
