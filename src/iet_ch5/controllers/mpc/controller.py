"""Constrained near-hover linear MPC, condensed QP solved by OSQP.

Physical wrench inputs obey the coupled polytope 0 <= inverse(allocation) @ nu <= 1.
No nonlinear formulation or recursive-feasibility/stability guarantee is asserted.
"""

import time

import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import block_diag

from ..lqr import LQR, hover_model, reference_vector, state_vector
from ...allocation import Allocator
from ...types import WrenchCommand


class MPC:
    def __init__(self, config):
        self.c = config
        self.reference_function = None

    def set_reference_source(self, function, current_time):
        self.reference_function, self.reference_time = function, current_time

    def reset(self, initial_state, reference, parameters):
        self.p = parameters
        self.allocator = Allocator(parameters)
        self.fallback = LQR(self.c)
        self.fallback.reset(initial_state, reference, parameters)
        self.A, self.B = hover_model(parameters, self.c["design_dt"])
        N = self.c["horizon"]
        self.Sx = np.vstack([np.linalg.matrix_power(self.A, i+1) for i in range(N)])
        self.Su = np.zeros((12*N, 4*N))
        for i in range(N):
            for j in range(i+1):
                self.Su[12*i:12*(i+1), 4*j:4*(j+1)] = np.linalg.matrix_power(self.A, i-j) @ self.B
        Qbar = block_diag(*([np.diag(self.c["Q"])]*(N-1) + [self.fallback.P]))
        Wbar = np.kron(np.eye(N), np.diag(self.c["W"]))
        self.linear_cost = 2*self.Su.T @ Qbar
        H = 2*(self.Su.T @ Qbar @ self.Su + Wbar)
        self.eq = np.array([parameters.mass * parameters.gravity, 0., 0., 0.])
        # Hard prediction constraints: full rotor coupling, speed box and small-angle tilt box.
        rotor_constraints = np.kron(np.eye(N), self.allocator.inverse)
        self.selector = np.zeros((5, 12))
        self.selector[:3, 3:6] = np.eye(3)
        self.selector[3, 6] = self.selector[4, 7] = 1
        self.Gx = np.kron(np.eye(N), self.selector)
        self.constraint_matrix = np.vstack([rotor_constraints, self.Gx @ self.Su])
        hover_pwm = self.allocator.inverse @ self.eq
        self.rotor_lo = np.tile(-hover_pwm, N)
        self.rotor_hi = np.tile(1 - hover_pwm, N)
        # Conservative boxes ensure Euclidean speed <= limit and small-angle tilt norm <= limit.
        self.state_bound = np.tile(np.r_[np.repeat(self.c["speed_limit"] / np.sqrt(3), 3),
                                         np.repeat(self.c["tilt_limit_rad"] / np.sqrt(2), 2)], N)
        self.optimizer = osqp.OSQP()
        self.optimizer.setup(P=sparse.csc_matrix(np.triu(H)), q=np.zeros(4*N),
            A=sparse.csc_matrix(self.constraint_matrix),
            l=np.r_[self.rotor_lo, -self.state_bound], u=np.r_[self.rotor_hi, self.state_bound],
            verbose=False, eps_abs=self.c["eps_abs"], eps_rel=self.c["eps_rel"],
            max_iter=self.c["max_iter"], time_limit=self.c["solver_time_limit_s"],
            polishing=False, warm_starting=True, adaptive_rho_interval=25)
        self.elapsed = 1e3
        self.last_command = WrenchCommand(self.eq[0], np.zeros(3))
        self.last_diagnostics = {}
        self.solution = None

    def update(self, state, reference, dt):
        self.elapsed += dt
        if self.elapsed < self.c["design_dt"] - 1e-7:
            return self.last_command, {**self.last_diagnostics, "qp_solved_this_update": False}
        self.elapsed = 0.
        start = time.perf_counter()
        x = state_vector(state)
        N = self.c["horizon"]
        if self.reference_function is None:
            references = [reference] * N
        else:
            references = [self.reference_function(self.reference_time+(i+1)*self.c["design_dt"])
                          for i in range(N)]
        desired = np.concatenate([reference_vector(r, self.p) for r in references])
        free = self.Sx @ x
        offsets = self.Gx @ free
        q = self.linear_cost @ (free - desired)
        lower, upper = np.r_[self.rotor_lo, -self.state_bound-offsets], np.r_[self.rotor_hi, self.state_bound-offsets]
        self.optimizer.update(q=q, l=lower, u=upper)
        if self.solution is not None:
            self.optimizer.warm_start(x=np.r_[self.solution[4:], self.solution[-4:]])
        result = self.optimizer.solve(raise_error=False)
        solver_ok = result.info.status_val == 1 and result.x is not None and np.isfinite(result.x).all()
        violation = None
        if solver_ok:
            predicted_constraints = self.constraint_matrix @ result.x
            violation = float(max(0., np.max(lower-predicted_constraints), np.max(predicted_constraints-upper)))
            solver_ok = violation <= self.c["constraint_acceptance_tolerance"]
        if solver_ok:
            self.solution = result.x.copy()
            nu = self.eq + result.x[:4]
            command = WrenchCommand(nu[0], nu[1:])
        else:
            command, _ = self.fallback.update(state, reference, dt)
            self.solution = None
        self.last_command = command
        self.last_diagnostics = {"qp_solved_this_update": True, "solver_status": result.info.status,
            "solver_status_value": result.info.status_val, "solver_iterations": result.info.iter,
            "solver_time_s": result.info.solve_time, "mpc_total_s": time.perf_counter()-start,
            "fallback": not solver_ok, "prediction_violation": violation,
            "preview_horizon": N, "prediction_dt": self.c["design_dt"]}
        return command, self.last_diagnostics.copy()
