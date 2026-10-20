import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from iet_ch5.controllers import make_controller
from iet_ch5.controllers.common import desired_attitude, unit_derivatives
from iet_ch5.controllers.lqr import hover_model
from iet_ch5.evaluation.metrics import settle_time, time_rms
from iet_ch5.references.tasks import Task
from iet_ch5.types import Reference, State, VehicleParameters


@pytest.fixture
def setup():
    c = json.loads(Path("configs/controllers.json").read_text())
    p = VehicleParameters(1, np.diag([.007, .008, .014]), 9.81,
        np.array([[.16, -.16, -.025], [-.16, .16, -.025], [.16, .16, -.025], [-.16, -.16, -.025]]),
        np.tile([0., 0., 1.], (4, 1)), np.array([-1, -1, 1, 1]), 4.2, .056, .99, .005, .003)
    state = State(np.array([0., 0., 2.]), np.zeros(3), np.eye(3), np.zeros(3), 1)
    return c, p, state


@pytest.mark.parametrize("name", ["pid", "lqr", "geometric"])
def test_equilibrium_reset_and_corrective_moments(setup, name):
    c, p, s = setup
    controller = make_controller(name, c[name])
    reference = Reference(s.p.copy())
    controller.reset(s, reference, p)
    hover, _ = controller.update(s, reference, .012)
    assert np.allclose(hover.vector(), [9.81, 0, 0, 0], atol=1e-9)
    for axis in range(3):
        s.R = Rotation.from_rotvec(np.eye(3)[axis] * .02).as_matrix()
        command, _ = controller.update(s, reference, .012)
        assert command.tau[axis] < 0
    s.R = np.eye(3)
    controller.reset(s, reference, p)
    assert np.allclose(controller.update(s, reference, .012)[0].vector(), hover.vector())


def test_lqr_closed_loop_and_coupled_sign(setup):
    c, p, s = setup
    controller = make_controller("lqr", c["lqr"])
    controller.reset(s, Reference(s.p), p)
    assert max(abs(np.linalg.eigvals(controller.A - controller.B @ controller.K))) < 1
    A, B = hover_model(p, .012)
    assert A[3, 7] > 0 and A[4, 6] < 0
    assert B[5, 0] > 0


def test_normalized_derivatives_and_singularities():
    x = np.array([.3, -.5, 2.])
    dx, ddx = np.array([.2, .1, -.1]), np.array([.1, -.2, .3])
    u, du, ddu = unit_derivatives(x, dx, ddx)
    h = 1e-4
    plus, minus = x + h * dx + h*h/2 * ddx, x - h * dx + h*h/2 * ddx
    plus, minus = plus / np.linalg.norm(plus), minus / np.linalg.norm(minus)
    assert np.allclose(du, (plus - minus) / (2*h), atol=1e-7)
    assert np.allclose(ddu, (plus - 2*u + minus) / h**2, atol=1e-7)
    for force in [np.zeros(3), [1., 0, 0]]:
        R, singular = desired_attitude(force, 0)
        assert singular
        assert np.allclose(R.T @ R, np.eye(3)) and np.linalg.det(R) > .999


@pytest.mark.parametrize("name", ["hover_transfer", "circle", "figure_eight"])
def test_reference_derivative_consistency(name):
    config = 'configs/figure-eight-demo-task.json' if name == 'figure_eight' else 'configs/tasks.json'
    task = Task(name, json.loads(Path(config).read_text()))
    t, h = 4.1, 1e-4
    ref, plus, minus = task.at(t), task.at(t+h), task.at(t-h)
    for field, derivative in [("p", "v"), ("v", "a"), ("a", "jerk"), ("jerk", "snap")]:
        assert np.allclose((getattr(plus, field)-getattr(minus, field))/(2*h),
                           getattr(ref, derivative), atol=1e-6)


def test_figure_eight_crossings_and_smooth_endpoints():
    from scipy.optimize import brentq
    from iet_ch5.references.tasks import SMOOTH

    config = json.loads(Path('configs/figure-eight-demo-task.json').read_text())
    task = Task('figure_eight', config)
    a, b = task.tracking_interval
    for t in [0., a, b, task.duration]:
        ref = task.at(t)
        assert np.allclose(ref.p, config['initial_position'], atol=1e-12)
        for field in ['v', 'a', 'jerk', 'snap']:
            assert np.allclose(getattr(ref, field), 0., atol=1e-12)
    crossings = []
    for progress in [.25, .75]:
        u = brentq(lambda u: SMOOTH(u) - progress, 0., 1.)
        ref = task.at(a + u * (b - a))
        assert np.allclose(ref.p, [-2., 0., 2.], atol=1e-10)
        crossings.append(ref.v)
    assert abs(np.cross(*crossings)[2]) > .5  # Distinct crossing tangents, not a retraced loop.
    h = 1e-4
    for t in [6.7, 11.3, 16.8, 21.4]:
        ref, plus, minus = task.at(t), task.at(t + h), task.at(t - h)
        for field, derivative in [('p', 'v'), ('v', 'a'), ('a', 'jerk'), ('jerk', 'snap')]:
            assert np.allclose((getattr(plus, field) - getattr(minus, field)) / (2*h),
                               getattr(ref, derivative), atol=2e-6)


def test_irregular_metrics_and_censored_settle():
    t = np.array([0., .1, .4, 1., 2.])
    assert time_rms(t, np.ones(5)*4) == pytest.approx(2)
    assert settle_time(t, np.ones(5), 0, .1, .5) is None
    assert settle_time(t, np.zeros(5), .4, .1, .5) == 0
