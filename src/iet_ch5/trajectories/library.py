"""Configurable SI/NWU reference library; analytic derivatives through snap."""

from importlib.resources import files
import json
from copy import deepcopy
from pathlib import Path
import numpy as np

from .timing import blend, cruise_phase, phase_harmonic
from ..types import Reference

FAMILIES = ('hover', 'transfer', 'circle', 'figure_eight', 'helix', 'spatial_lissajous')


class Trajectory:
    def __init__(self, config):
        unknown = set(config)-{'schema','id','family','tier','description','initial_position','timing','shape','heading','evaluation','safety','camera_position_nwu_m'}
        if unknown:
            raise ValueError(f'Unknown trajectory fields: {sorted(unknown)}')
        for section in ('timing', 'shape', 'heading', 'evaluation', 'safety'):
            if not all(np.issubdtype(np.asarray(v).dtype, np.number) and np.isfinite(v).all() for v in config[section].values()):
                raise ValueError(f'Expected finite numeric values in {section}')
        for section in ('evaluation', 'safety'):
            if min(config[section].values()) <= 0:
                raise ValueError(f'Expected positive limits in {section}')
        self.c = deepcopy(config)
        self.name = config['id']
        self.family = config['family']
        if self.family not in FAMILIES:
            raise ValueError(f'Unknown trajectory family: {self.family}')
        self.origin = np.array(config['initial_position'], dtype=float)
        self.timing = config['timing']
        t = self.timing
        if self.origin.shape != (3,) or not np.isfinite(self.origin).all():
            raise ValueError('initial_position must contain three finite SI coordinates')
        if min(t['onset_s'], t['final_hold_s']) < 0 or t['duration_s'] <= 0:
            raise ValueError('Invalid trajectory interval')
        if self.family not in ('hover', 'transfer'):
            cruise_phase(0., 0., t['duration_s'], t['ramp_s'])  # Validate before simulator actions.
        self.duration = t['onset_s'] + t['duration_s'] + t['final_hold_s']
        self.tracking_interval = (t['onset_s'], t['onset_s']+t['duration_s'])

    def phase_at(self, t):
        return cruise_phase(t, self.timing['onset_s'], self.timing['duration_s'], self.timing['ramp_s'])

    def at(self, t):
        c, s = self.c, self.c['shape']
        values = np.zeros((5, 3))
        values[0] = self.origin
        heading = c['heading']['offset_rad']
        dh = ddh = 0.
        if self.family == 'hover':
            return Reference(*values, heading=heading)
        if self.family == 'transfer':
            b = blend(t, self.timing['onset_s'], self.timing['duration_s'])
            values += np.outer(b, np.array(s['displacement_m']))
            return Reference(*values, heading=heading)
        phase = self.phase_at(t)
        harmonics = {i: phase_harmonic(phase, i) for i in [1, 2, 3]}
        if self.family in ('circle', 'helix'):
            terms = [(0, 'cos', 1, s['radius_m']), (1, 'sin', 1, s['radius_m'])]
        else:
            terms = [(0, 'cos', 1, s['half_width_m']), (1, 'sin', 2, s['half_height_m'])]
        if self.family == 'spatial_lissajous':
            terms += [(2, 'sin', 1, s['vertical_amplitude_m']),
                      (1, 'sin', 3, s['third_harmonic_y_m']),
                      (2, 'sin', 3, s['third_harmonic_z_m'])]
        for axis, basis, frequency, amplitude in terms:
            v = np.array(harmonics[frequency])
            v = v.real if basis == 'cos' else v.imag
            v = v.copy()
            if basis == 'cos':
                v[0] -= 1.
            values[:, axis] += amplitude*v
        if self.family == 'helix':
            values[:, 2] += s['climb_m']*phase/(2*np.pi)
        h = c['heading']['oscillation_rad']*np.array(harmonics[1]).imag
        heading, dh, ddh = heading+h[0], h[1], h[2]
        return Reference(*values, heading=heading, heading_rate=dh, heading_acceleration=ddh)


def available_trajectories():
    return sorted(p.name[:-5] for p in files('iet_ch5.trajectories').joinpath('configs').iterdir()
                  if p.name.endswith('.json'))


def load_trajectory(name_or_path):
    path = Path(name_or_path)
    if path.exists():
        config = json.loads(path.read_text())
    else:
        if name_or_path not in available_trajectories():
            raise ValueError(f'Unknown trajectory: {name_or_path}')
        config = json.loads(files('iet_ch5.trajectories').joinpath('configs', name_or_path+'.json').read_text())
    Trajectory(config)
    return config
