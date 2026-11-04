import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from iet_ch5.allocation import Allocator
from iet_ch5.frames import C, hat, parameters_from_rpc, rotation, state_from_sample, vee
from iet_ch5.types import VehicleParameters, WrenchCommand


@pytest.fixture
def vehicle():
    # Synthetic numerical fixture, never used as a simulator parameter fallback.
    return VehicleParameters(1, np.diag([.007, .008, .014]), 9.81,
                             np.array([[.16, -.16, -.025], [-.16, .16, -.025],
                                       [.16, .16, -.025], [-.16, -.16, -.025]]),
                             np.tile([0., 0., 1.], (4, 1)), np.array([-1, -1, 1, 1]),
                             4.2, .056, .99, .005, .003)


def test_nonzero_attitude_and_vector_round_trips():
    r = Rotation.from_euler("xyz", [.32, -.21, 1.12]).as_matrix()
    assert np.linalg.det(C) == 1
    assert np.allclose(C @ (C @ r @ C) @ C, r)
    v = np.array([.2, -.8, 1.5])
    assert np.allclose((C @ r @ C) @ (C @ v), C @ (r @ v))
    assert np.allclose(C @ np.cross(v, [2, 3, 4]), np.cross(C @ v, C @ [2, 3, 4]))
    j = np.array([[2., .2, -.1], [.2, 3., .3], [-.1, .3, 4.]])
    assert np.allclose((C @ j @ C) @ (C @ v), C @ (j @ v))
    assert np.allclose(C @ hat(v) @ C, hat(C @ v))
    assert np.allclose(vee(hat(v)), v)
    q = Rotation.from_matrix(r).as_quat()
    assert np.allclose(rotation(dict(zip(['x_val','y_val','z_val','w_val'], q))), r)


def test_yaw_sign_and_body_rate():
    q = Rotation.from_euler("z", .7).as_quat()
    vec = dict(zip(['x_val','y_val','z_val'], [1., 2., 3.]))
    sample = {"timestamp": 42, "kinematics": {"position": vec, "linear_velocity": vec,
              "angular_velocity": vec, "orientation": dict(zip(
                  ['x_val','y_val','z_val','w_val'], q))}}
    s = state_from_sample(sample)
    assert np.allclose(s.p, [1, -2, -3])
    assert np.allclose(s.v, s.p)
    assert np.allclose(s.omega, s.p)
    assert Rotation.from_matrix(s.R).as_euler("xyz")[2] == pytest.approx(-.7)


def test_hover_and_moment_signs(vehicle):
    a = Allocator(vehicle)
    hover = a.allocate(WrenchCommand(9.81, np.zeros(3)))
    assert np.allclose(hover.predicted_wrench, [9.81, 0, 0, 0])
    assert np.allclose(hover.pwm, hover.pwm[0])
    # Positive roll: left rotors 1,2. Positive pitch: rear rotors 1,3.
    assert np.array_equal(np.sign(a.B[1]), [-1, 1, 1, -1])
    assert np.array_equal(np.sign(a.B[2]), [-1, 1, -1, 1])
    assert np.array_equal(np.sign(a.B[3]), [-1, -1, 1, 1])
    for axis in range(3):
        wrench = np.r_[9.81, np.eye(3)[axis] * .01]
        out = a.allocate(WrenchCommand(wrench[0], wrench[1:]))
        assert np.allclose(out.predicted_wrench, wrench)


def test_feasible_bounds_and_bad_input(vehicle):
    a = Allocator(vehicle)
    out = a.allocate(WrenchCommand(100, np.array([1., -2., 3.])))
    assert np.all((out.pwm >= 0) & (out.pwm <= 1))
    assert np.linalg.norm(out.residual) > 1
    assert out.saturated.any()
    with pytest.raises(ValueError):
        a.allocate(WrenchCommand(float('nan'), np.zeros(3)))
    with pytest.raises(KeyError):
        parameters_from_rpc({})
