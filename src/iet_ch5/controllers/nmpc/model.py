"""Original NWU/FLU rigid-body model with quaternion attitude and measured motor lag.

x = [p(3), v(3), quaternion_wxyz(4), body_omega(3), applied_wrench(4)].
u = commanded [collective_N, body_moments_Nm], before the one shared allocator.
All motors have the same measured first-order lag, so the constant linear allocation
maps rotor lag exactly to applied-wrench lag at the nominal density.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def quaternion_from_rotation(R):
    xyzw = Rotation.from_matrix(R).as_quat()
    return np.r_[xyzw[3], xyzw[:3]]


def rotation_from_quaternion(q):
    q = np.asarray(q, dtype=float)
    if np.linalg.norm(q) < 1e-10:
        raise ValueError('Zero quaternion')
    return Rotation.from_quat(np.r_[q[1:], q[0]]).as_matrix()


def state_vector(state, applied):
    return np.r_[state.p, state.v, quaternion_from_rotation(state.R), state.omega, applied]


def dynamics(x, command, parameters):
    """Independent NumPy RHS for model checks and numerical replay, not AirSim truth."""
    p = parameters
    q, omega, applied = x[6:10], x[10:13], x[13:17]
    acceleration = applied[0]/p.mass*rotation_from_quaternion(q)[:, 2]-np.array([0., 0., p.gravity])
    qdot = .5*np.r_[-q[1:]@omega, q[0]*omega+np.cross(q[1:], omega)]
    domega = np.linalg.solve(p.J, applied[1:]-np.cross(omega, p.J@omega))
    return np.r_[x[3:6], acceleration, qdot, domega,
                 (np.asarray(command)-applied)/p.motor_time_constant]


def symbolic_model(parameters, name):
    import casadi as ca
    from acados_template import AcadosModel
    p = parameters
    x, u, reference = ca.SX.sym('x', 17), ca.SX.sym('u', 4), ca.SX.sym('reference', 17)
    def rotation(q):
        q = q/ca.sqrt(ca.dot(q, q)+1e-16)
        w, a, b, c = (q[i] for i in range(4))
        return ca.vertcat(ca.horzcat(1-2*(b*b+c*c), 2*(a*b-w*c), 2*(a*c+w*b)),
                          ca.horzcat(2*(a*b+w*c), 1-2*(a*a+c*c), 2*(b*c-w*a)),
                          ca.horzcat(2*(a*c-w*b), 2*(b*c+w*a), 1-2*(a*a+b*b)))
    q, omega, applied = x[6:10], x[10:13], x[13:17]
    R, Rd = rotation(q), rotation(reference[6:10])
    qdot = .5*ca.vertcat(-ca.dot(q[1:], omega), q[0]*omega+ca.cross(q[1:], omega))
    rhs = ca.vertcat(x[3:6], applied[0]/p.mass*R[:, 2]-ca.vertcat(0, 0, p.gravity),
                     qdot, ca.solve(ca.DM(p.J), applied[1:]-ca.cross(omega, ca.DM(p.J)@omega)),
                     (u-applied)/p.motor_time_constant)
    qn, qr = q/ca.sqrt(ca.dot(q, q)+1e-16), reference[6:10]
    attitude_error = 2*(qr[0]*qn[1:]-qn[0]*qr[1:]-ca.cross(qr[1:], qn[1:]))
    rate_error = omega-R.T@Rd@reference[10:13]
    state_error = ca.vertcat(x[:3]-reference[:3], x[3:6]-reference[3:6],
                            attitude_error, rate_error, applied-reference[13:17])
    model = AcadosModel()
    model.name, model.x, model.u, model.p = name, x, u, reference
    model.xdot = ca.SX.sym('xdot', 17)
    model.f_expl_expr, model.f_impl_expr = rhs, model.xdot-rhs
    model.cost_y_expr = ca.vertcat(state_error, u-reference[13:17])
    model.cost_y_expr_e = state_error
    return model
