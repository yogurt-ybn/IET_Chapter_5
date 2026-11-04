"""New trajectory calculus, frozen defaults, failure handling and repeatability checks."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from iet_ch5.controllers.configuration import load_controller_configs
from iet_ch5.experiments.configuration import resolve
from iet_ch5.experiments.engine import classify, execute
from iet_ch5.experiments.publication import repeatability
from iet_ch5.trajectories import Trajectory, available_trajectories, load_trajectory


@pytest.mark.parametrize('name', available_trajectories())
def test_analytic_trajectory_derivatives_and_endpoint_hold(name):
    task = Trajectory(load_trajectory(name))
    a, b = task.tracking_interval
    for t in np.linspace(a+.17, b-.17, 9):
        h = 1e-5
        ref, plus, minus = task.at(t), task.at(t+h), task.at(t-h)
        for field, derivative in [('p', 'v'), ('v', 'a'), ('a', 'jerk'), ('jerk', 'snap')]:
            assert np.allclose((getattr(plus, field)-getattr(minus, field))/(2*h), getattr(ref, derivative), atol=2e-6)
        assert (plus.heading-minus.heading)/(2*h) == pytest.approx(ref.heading_rate, abs=1e-7)
        assert (plus.heading_rate-minus.heading_rate)/(2*h) == pytest.approx(ref.heading_acceleration, abs=1e-7)
    for t in [0, a, b, task.duration]:
        ref = task.at(t)
        for field in ['v', 'a', 'jerk', 'snap']:
            assert np.allclose(getattr(ref, field), 0, atol=1e-10)
    final = task.at(task.duration).p
    if name == 'helix':
        assert final[2]-task.origin[2] == pytest.approx(1.)
    elif name == 'transfer':
        assert np.allclose(final-task.origin, task.c['shape']['displacement_m'])
    else:
        assert np.allclose(final, task.origin)


def test_owned_defaults_preserve_original_numerical_configuration():
    legacy = json.loads(Path('configs/controllers.json').read_text())
    assert {k: load_controller_configs()[k] for k in legacy} == legacy
    override = {'dob': {'nominal': {'position_kp': [5., 5., 6.]}}}
    changed = load_controller_configs(overrides=override)
    assert changed['dob']['nominal']['position_kp'] == [5., 5., 6.]
    assert {k: load_controller_configs()[k] for k in legacy} == legacy
    for bad in [{'pid': {'unknown_gain': 1}}, {'mpc': {'horizon': 1.5}}, {'pid': {'velocity_limit': -1}},
                {'pid': {'position_kp': [1., 2.]}}, {'dob': {'nominal': []}}, {'pid': {'velocity_limit': float('nan')}}]:
        with pytest.raises(ValueError):
            load_controller_configs(overrides=bad)


def test_invalid_trajectory_rejected_before_simulator_access():
    c = load_trajectory('agile_course')
    c['timing']['ramp_s'] = 10
    with pytest.raises(ValueError):
        Trajectory(c)
    c = load_trajectory('hover')
    c['active_duration_s'] = 5
    with pytest.raises(ValueError):
        Trajectory(c)


def test_matrix_freezes_paired_repeats_and_noise_protocol():
    plan = resolve('configs/experiments/case_study.json')
    assert len(plan['matrix']) == 70
    assert len({r['run_id'] for r in plan['matrix']}) == 70
    noisy = resolve('configs/experiments/robustness.json')
    seeds = {r['seed'] for r in noisy['matrix'] if r['condition'] == 'noise_delay'}
    assert seeds == set(range(5))
    assert len(plan['matrix'][0]['task_config']['trajectory']['initial_position']) == 3


def test_incomplete_rms_never_classified_as_pass_and_repeatability_is_censored():
    passed = SimpleNamespace(status='passed')
    assert classify(passed, {'tracking_rms_m': 0, 'complete_common_interval': False}) == 'aborted'
    rows = [dict(controller='pid', trajectory='circle', condition='nominal', seed=0, repeat=i,
                 run_id=str(i), metrics={'complete_common_interval': i == 0}) for i in range(2)]
    data = {str(i): {'t': np.array([0., 1.]), 'truth_p': np.zeros((2, 3))} for i in range(2)}
    report = repeatability(rows, data, .02)[0]
    assert report['max_position_delta_m'] == 0 and not report['passed']


def test_resume_keeps_complete_tracking_failures_and_preserves_incomplete_attempt(tmp_path, monkeypatch):
    import iet_ch5.experiments.engine as engine
    plan = resolve('configs/experiments/case_study.json', controllers=['geometric'], trajectories=['hover'], repeats=1)
    fake = SimpleNamespace(parameters=None, close=lambda: None)
    monkeypatch.setattr(engine, 'create_backend', lambda c: fake)
    monkeypatch.setattr(engine, 'screen', lambda *a: {'physical_feedforward_feasible': True})
    calls = []
    def fake_run(*args, **kwargs):
        path = args[5]
        path.mkdir(parents=True)
        (path/'sentinel').write_text('retained')
        calls.append(path)
        return SimpleNamespace(status='failed', failure='Tracking acceptance tolerance not met', metrics={})
    monkeypatch.setattr(engine, 'run', fake_run)
    monkeypatch.setattr(engine, 'reanalyze', lambda *a: ({'complete_common_interval': True, 'tracking_rms_m': .3}, None))
    output = tmp_path/'suite'
    journal = execute(plan, {}, output)
    key = plan['matrix'][0]['run_id']
    assert journal[key]['status'] == 'tracking_failed'
    execute(plan, {}, output, resume=True)
    assert len(calls) == 1
    journal[key]['status'] = 'interrupted'
    engine.atomic_json(output/'progress.json', journal)
    execute(plan, {}, output, resume=True)
    assert len(calls) == 2
    attempts = list((output/'attempts').iterdir())
    assert len(attempts) == 1 and (attempts[0]/'sentinel').read_text() == 'retained'
    changed = resolve('configs/experiments/case_study.json', controllers=['pid'], trajectories=['hover'], repeats=1)
    with pytest.raises(ValueError, match='identical'):
        execute(changed, {}, output, resume=True)


def test_publication_all_aborted_runs_retains_censoring_without_timing(tmp_path):
    pytest.importorskip('matplotlib', reason='Publication QA requires optional plots dependency')
    from iet_ch5.experiments.publication import plot
    row = dict(controller='pid', trajectory='circle', condition='nominal', seed=0, repeat=0,
               run_id='aborted', status='aborted',
               metrics={'complete_common_interval': False, 'tracking_rms_m': None})
    d = {'t': np.array([0., .012]), 'truth_p': np.zeros((2, 3)), 'reference_p': np.zeros((2, 3)),
         'truth_v': np.zeros((2, 3)), 'requested_wrench': np.zeros((2, 4)),
         'telemetry_estimated_wrench': np.zeros((2, 4)), 'motor_pwm': np.zeros((2, 4))}
    plot([row], {'aborted': d}, tmp_path/'figures')
    assert (tmp_path/'figures'/'accuracy_matrix.png').exists()
    assert not row['metrics']['complete_common_interval'] and row['metrics']['tracking_rms_m'] is None
