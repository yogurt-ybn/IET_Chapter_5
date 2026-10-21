"""Resolve human-facing trajectory/controller/condition configs into one frozen protocol."""

import json
from pathlib import Path
from ..controllers.configuration import load_controller_configs, METHODS
from ..trajectories import load_trajectory, Trajectory


def build_runtime(trajectory, condition, common):
    task = Trajectory(trajectory)
    a, b = task.tracking_interval
    evaluation, safety = trajectory['evaluation'], trajectory['safety']
    wind_onset = a+condition.get('onset_fraction', .4)*(b-a)
    return {'trajectory': trajectory, 'condition': condition, 'duration': task.duration,
            'initial_position': trajectory['initial_position'],
            'control_period': common['control_period_s'],
            'transfer_onset': a, 'transfer_duration': b-a,
            'settle_tolerance': common['settling_tolerance_m'], 'settle_dwell': common['settling_dwell_s'],
            'hold_acceptance_rms_m': evaluation['position_rms_limit_m'],
            'evaluation_speed_limit': evaluation['speed_limit_m_s'],
            'evaluation_tilt_limit': evaluation['tilt_limit_rad'],
            'abort_position_error': safety['max_position_error_m'],
            'abort_speed': safety['speed_limit_m_s'], 'abort_tilt_rad': safety['tilt_limit_rad'],
            'feedback': common['feedback'], 'controller_mass_factor': condition.get('mass_factor', 1.1),
            'controller_inertia_factor': condition.get('inertia_factor', 1.15),
            'disturbance_onset': wind_onset, 'disturbance_duration': condition.get('duration_s', 1.5),
            'wind_velocity_nwu_m_s': condition.get('velocity_nwu_m_s', [8., -4., 0.])}


def resolve(suite_path, *, controllers=None, trajectories=None, repeats=None, controller_overrides=None,
            runtime_path='configs/experiments/runtime.json'):
    suite = json.loads(Path(suite_path).read_text())
    if controllers is not None:
        suite['controllers'] = list(METHODS) if controllers == ['all'] else controllers
    if trajectories is not None:
        from ..trajectories import available_trajectories
        suite['trajectories'] = available_trajectories() if trajectories == ['all'] else trajectories
    if repeats is not None:
        suite['repeats'] = repeats
    if not isinstance(suite['repeats'], int) or suite['repeats'] < 1:
        raise ValueError('repeats must be a positive integer')
    if not suite['controllers'] or len(set(suite['controllers'])) != len(suite['controllers']) or not suite['trajectories']:
        raise ValueError('Nonempty unique controllers and nonempty trajectories required')
    if any(m not in METHODS for m in suite['controllers']):
        raise ValueError('Unknown controller')
    docs = [load_trajectory(t) for t in suite['trajectories']]
    if len({d['id'] for d in docs}) != len(docs):
        raise ValueError('Trajectory IDs must be unique')
    common = json.loads(Path(runtime_path).read_text())
    if not 0 < common['control_period_s'] < .1:
        raise ValueError('Control period outside verified bridge range')
    overrides = json.loads(Path(controller_overrides).read_text()) if controller_overrides else None
    parameters = load_controller_configs(overrides=overrides)
    matrix = []
    for doc in docs:
        for condition in suite['conditions']:
            if condition['kind'] not in ('nominal', 'wind', 'mismatch', 'noise_delay'):
                raise ValueError('Unsupported condition')
            if condition['kind'] == 'noise_delay' and len(set(condition['seeds'])) < 5:
                raise ValueError('Noise/delay comparisons require at least five paired seeds')
            runtime = build_runtime(doc, condition, common)
            for method in suite['controllers']:
                for seed in condition.get('seeds', [0]):
                    for repeat in range(suite['repeats']):
                        matrix.append({'controller': method, 'trajectory': doc['id'], 'tier': doc['tier'],
                                       'condition': condition['id'], 'seed': seed, 'repeat': repeat,
                                       'task_config': runtime})
    for i, item in enumerate(matrix):
        item['run_id'] = f"{i:03d}-{item['controller']}-{item['trajectory']}-{item['condition']}-s{item['seed']}-r{item['repeat']}"
    return {'suite': suite, 'controllers': parameters, 'trajectories': docs,
            'runtime': common, 'matrix': matrix}
