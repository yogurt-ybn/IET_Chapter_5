import copy
import json
from pathlib import Path

import numpy as np

from iet_ch5.allocation import Allocator
from iet_ch5.controllers import make_controller
from iet_ch5.types import Reference, State, VehicleParameters


def setup():
    c = json.loads(Path('configs/controllers.json').read_text())
    p = VehicleParameters(1, np.diag([.007, .008, .014]), 9.81,
        np.array([[.16, -.16, -.025], [-.16, .16, -.025], [.16, .16, -.025], [-.16, -.16, -.025]]),
        np.tile([0., 0., 1.], (4, 1)), np.array([-1, -1, 1, 1]), 4.2, .056, .99, .005, .003)
    s = State(np.array([0., 0., 2.]), np.zeros(3), np.eye(3), np.zeros(3), 1)
    return c, p, s, Reference(s.p.copy())


def test_force_observer_convergence_bounds_and_identical_disabled_nominal():
    c, p, s, ref = setup()
    dob = make_controller('dob', c['dob'])
    dob.reset(s, ref, p)
    for _ in range(200):
        s.v[0] += .6 * .012
        dob.update(s, ref, .012)
    assert abs(dob.estimate[0] - .6) < 1e-4
    for _ in range(100):
        s.v[0] += 10 * .012
        dob.update(s, ref, .012)
    assert abs(dob.estimate[0]) <= c['dob']['force_bound_N']
    off = copy.deepcopy(c['dob'])
    off['enabled'] = False
    disabled, nominal = make_controller('dob', off), make_controller('geometric', c['geometric'])
    disabled.reset(s, ref, p)
    nominal.reset(s, ref, p)
    s.v[0] += .3
    assert np.allclose(disabled.update(s, ref, .012)[0].vector(), nominal.update(s, ref, .012)[0].vector())


def test_mpc_solves_real_qp_and_preserves_coupled_feasible_input():
    c, p, s, ref = setup()
    config = {**c['mpc'], 'solver_time_limit_s': 1.0}
    mpc = make_controller('mpc', config)
    mpc.reset(s, ref, p)
    ref.p += np.array([1., .5, .3])
    wrench, diag = mpc.update(s, ref, .036)
    assert not diag['fallback'] and diag['solver_status_value'] == 1
    assert diag['prediction_violation'] <= config['constraint_acceptance_tolerance']
    pwm = Allocator(p).inverse @ wrench.vector()
    assert np.all(pwm >= -1e-4) and np.all(pwm <= 1+1e-4)
    assert mpc.solution is not None and len(mpc.solution) == 4*config['horizon']


def test_mpc_timeout_and_infeasible_state_trigger_explicit_fallback():
    c, p, s, ref = setup()
    for config, velocity in [({**c['mpc'], 'solver_time_limit_s': 1e-9}, 0.),
                             ({**c['mpc'], 'solver_time_limit_s': 1.0}, 20.)]:
        mpc = make_controller('mpc', config)
        mpc.reset(s, ref, p)
        s.v[0] = velocity
        ref.p[0] = 2.
        wrench, diag = mpc.update(s, ref, .036)
        assert diag['fallback'] and diag['solver_status_value'] != 1
        allocated = Allocator(p).allocate(wrench)
        assert np.isfinite(allocated.pwm).all() and np.all((allocated.pwm >= 0) & (allocated.pwm <= 1))
