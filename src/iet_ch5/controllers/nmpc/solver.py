"""Pinned acados C-code generation/cache; no code generation in evaluated updates."""
import ctypes
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path

import numpy as np

from ...allocation import Allocator
from ...logging import dump
from .model import symbolic_model

ACADOS_COMMIT = 'd9d0ba2e8f127855119d38874af8a9570df1d8e5'


def create_solver(config, parameters, x0):
    try:
        from acados_template import AcadosOcp, AcadosOcpSolver
        from acados_template.utils import get_acados_path
    except ImportError as error:
        raise RuntimeError('NMPC requires the pinned acados installation; run scripts/install_acados.py') from error
    root = Path(get_acados_path())
    os.environ.setdefault('ACADOS_SOURCE_DIR', str(root))
    # Source is pinned by installation receipt. No silent substitution of a different build.
    receipt_path = root/'iet_ch5_installation.json'
    if not receipt_path.exists():
        raise RuntimeError('Missing acados installation receipt; run scripts/install_acados.py')
    receipt = json.loads(receipt_path.read_text())
    if receipt['acados_commit'] != ACADOS_COMMIT:
        raise RuntimeError('Unexpected acados source revision')
    for name in ('blasfeo', 'hpipm', 'acados'):
        library = root/'lib'/f'lib{name}.so'
        if hashlib.sha256(library.read_bytes()).hexdigest() != receipt['library_sha256'][name]:
            raise RuntimeError(f'acados dependency binary changed: {name}')
        ctypes.CDLL(str(library), mode=ctypes.RTLD_GLOBAL)
    fingerprint = hashlib.sha256(dump({'config': config, 'parameters': asdict(parameters),
        'receipt': receipt}).encode())
    for path in sorted(Path(__file__).parent.glob('*.py')):
        fingerprint.update(path.read_bytes())
    key = fingerprint.hexdigest()[:16]
    cache = Path(os.environ.get('IET_CH5_ACADOS_CACHE', '.local/acados-generated')).resolve()/key
    cache.mkdir(parents=True, exist_ok=True)
    N = config['horizon']
    ocp = AcadosOcp()
    ocp.model = symbolic_model(parameters, 'ch5_rigid_'+key)
    ocp.code_export_directory = str(cache/'c_generated_code')
    ocp.parameter_values = x0
    state_weights = np.r_[config['position_weight'], config['velocity_weight'], config['attitude_weight'],
                          config['rate_weight'], config['applied_wrench_weight']]
    ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.W = np.diag(np.r_[state_weights, config['command_wrench_weight']])
    ocp.cost.W_e = np.diag(config['terminal_scale']*state_weights)
    ocp.cost.yref, ocp.cost.yref_e = np.zeros(20), np.zeros(16)
    # Exact coupled rotor-input polytope on physical commanded thrust/moments.
    ocp.constraints.C = np.zeros((4, 17))
    ocp.constraints.D = Allocator(parameters).inverse
    ocp.constraints.lg, ocp.constraints.ug = np.zeros(4), np.ones(4)
    ocp.constraints.x0 = x0
    opts = ocp.solver_options
    opts.N_horizon, opts.tf = N, N*config['design_dt']
    opts.nlp_solver_type = config['solver_type']
    opts.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    opts.qp_solver_cond_N = config['condensing_stages']
    opts.hessian_approx, opts.integrator_type = 'GAUSS_NEWTON', 'ERK'
    opts.sim_method_num_stages, opts.sim_method_num_steps = 4, config['integration_steps']
    opts.qp_solver_iter_max, opts.nlp_solver_max_iter = config['qp_max_iter'], config['max_iter']
    opts.nlp_solver_tol_stat = opts.nlp_solver_tol_eq = config['solver_tolerance']
    opts.nlp_solver_tol_ineq = opts.nlp_solver_tol_comp = config['solver_tolerance']
    opts.qp_solver_warm_start = 1 if config['warm_start'] else 0
    opts.nlp_solver_warm_start_first_qp = config['warm_start']
    opts.print_level = 0
    filename = cache/'ocp.json'
    built = (cache/'build_receipt.json').exists()
    solver = AcadosOcpSolver(ocp, json_file=str(filename), generate=not built, build=not built,
                             verbose=False)
    binary = cache/'c_generated_code'/f'libacados_ocp_solver_{ocp.model.name}.so'
    provenance = {'model': '17-state nonlinear rigid body + common motor lag; NWU/FLU',
                  'acados': receipt, 'generated_solver_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
                  'cache_key': key, 'solver_type': config['solver_type'],
                  'qp_solver': opts.qp_solver, 'condensing_stages': opts.qp_solver_cond_N,
                  'horizon_s': opts.tf, 'state_order': 'p,v,q_wxyz,body_omega,applied_wrench',
                  'constraints': 'coupled motor input [0,1]; no hard speed/tilt state caps',
                  'stability_claim': 'none; finite horizon least-squares terminal cost'}
    (cache/'build_receipt.json').write_text(dump(provenance)+'\n')
    return solver, provenance
