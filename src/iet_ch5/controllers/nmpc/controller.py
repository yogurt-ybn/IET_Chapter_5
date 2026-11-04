"""Full rigid-body NMPC, generated acados SQP-RTI / HPIPM; geometric fallback is explicit."""
import time
import numpy as np

from ..geometric import Geometric
from ...allocation import Allocator
from ...types import WrenchCommand
from .model import state_vector, rotation_from_quaternion
from .reference import ReferenceTable, flat_reference
from .solver import create_solver


class NMPC:
    def __init__(self, config):
        self.c = config

    def reset(self, initial_state, reference, parameters):
        self.p = parameters
        self.allocator = Allocator(parameters)
        self.applied = np.r_[parameters.mass*parameters.gravity, np.zeros(3)]
        self.solver, self.provenance = create_solver(self.c, parameters, state_vector(initial_state, self.applied))
        self.fallback = Geometric(self.c['fallback'])
        self.fallback.reset(initial_state, reference, parameters)
        self.elapsed, self.reference_time = 1e3, 0.
        self.table = None
        self.solution = None
        self.last_command = WrenchCommand(self.applied[0], self.applied[1:])
        self.last_diagnostics = {}

    def prepare_reference(self, function, duration):
        started = time.perf_counter()
        self.table = ReferenceTable(function, duration+self.c['horizon']*self.c['design_dt'], self.p,
                                    self.c['reference_table_period_s'])
        self.provenance['reference_preparation_s'] = time.perf_counter()-started
        self.provenance['reference_interpolation'] = f"{self.c['reference_table_period_s']} s analytic flatness table; linear interpolation with quaternion normalization, timed online"

    def set_reference_source(self, function, current_time):
        self.reference_time = current_time

    def set_applied_wrench(self, applied):
        self.applied = np.asarray(applied).copy()

    def update(self, state, reference, dt):
        self.elapsed += dt
        if self.elapsed < self.c['design_dt']-1e-7:
            return self.last_command, {**self.last_diagnostics, 'qp_solved_this_update': False}
        self.elapsed = 0.
        N, period = self.c['horizon'], self.c['design_dt']
        times = self.reference_time+np.arange(N+1)*period
        refs = self.table.at(times) if self.table is not None else np.tile(flat_reference(reference, self.p), (N+1, 1))
        x0 = state_vector(state, self.applied)
        if x0[6:10]@refs[0, 6:10] < 0:
            x0[6:10] *= -1
        if self.solution is None or not self.c['warm_start']:
            self.solver.reset()
            guesses = refs
            inputs = refs[:-1, 13:17]
        else:
            xs, us = self.solution
            guesses = np.vstack([xs[1:], xs[-1]])
            inputs = np.vstack([us[1:], us[-1]])
        for k in range(N+1):
            self.solver.set(k, 'p', refs[k])
            self.solver.set(k, 'x', x0 if k == 0 else guesses[k])
            if k < N:
                self.solver.set(k, 'u', inputs[k])
        self.solver.constraints_set(0, 'lbx', x0)
        self.solver.constraints_set(0, 'ubx', x0)
        started = time.perf_counter()
        status = self.solver.solve()
        solve_wall = time.perf_counter()-started
        candidate = self.solver.get(0, 'u')
        pwm = self.allocator.inverse@candidate
        violation = float(max(0., np.max(-pwm), np.max(pwm-1))) if np.isfinite(pwm).all() else None
        accepted = status == 0 and violation is not None and violation <= self.c['constraint_acceptance_tolerance']
        if accepted:
            xs = np.array([self.solver.get(k, 'x') for k in range(N+1)])
            us = np.array([self.solver.get(k, 'u') for k in range(N)])
            accepted = np.isfinite(xs).all() and np.isfinite(us).all()
        if accepted:
            self.solution = xs, us
            command = WrenchCommand(float(candidate[0]), candidate[1:].copy())
        else:
            command, _ = self.fallback.update(state, reference, dt)
            self.solution = None
        self.last_command = command
        residuals = self.solver.get_residuals(recompute=True)
        self.last_diagnostics = {'qp_solved_this_update': True, 'solver_status': f'acados_{status}',
            'solver_status_value': int(status), 'solver_type': self.c['solver_type'],
            'solver_time_s': float(self.solver.get_stats('time_tot')), 'solver_call_wall_s': solve_wall,
            'solver_iterations': int(self.solver.get_stats('sqp_iter')),
            'nlp_residuals_stationarity_dynamics_inequality_complementarity': [float(v) if np.isfinite(v) else None for v in residuals],
            'predicted_quaternion_norm_error': float(np.max(np.abs(np.linalg.norm(xs[:, 6:10], axis=1)-1))) if accepted else None,
            'qp_iterations': np.asarray(self.solver.get_stats('qp_iter')).tolist(),
            'fallback': not bool(accepted), 'fallback_controller': 'geometric',
            'prediction_violation': violation, 'desired_R': rotation_from_quaternion(refs[0, 6:10]),
            'desired_rate': refs[0, 10:13], 'prediction_dt': period, 'preview_horizon': N}
        return command, self.last_diagnostics.copy()
