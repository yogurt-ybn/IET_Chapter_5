#!/usr/bin/env python3
"""Replay measured feedback through identical rigid-body OCP variants; not a flight benchmark."""
import argparse
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from iet_ch5.controllers.configuration import load_controller_configs
from iet_ch5.controllers.nmpc import NMPC
from iet_ch5.experiments.engine import atomic_json, source_fingerprint
from iet_ch5.logging import revision
from iet_ch5.references.tasks import Task
from iet_ch5.types import State, VehicleParameters


def decode(value):
    return State(**{k: np.array(v) if k in ('p', 'v', 'R', 'omega') else v for k, v in value.items()})


def benchmark(source, output, repeats):
    source, output = Path(source), Path(output)
    meta = json.loads((source/'metadata.json').read_text())
    records = [json.loads(s) for s in (source/'log.jsonl').read_text().splitlines()]
    records = [r for r in records if r['diagnostics'].get('qp_solved_this_update')]
    p = VehicleParameters(**{k: np.array(v) if isinstance(v, list) else v for k, v in meta['parameters'].items()})
    task = Task(meta['config']['task'], meta['config']['task_config'])
    base = load_controller_configs()['nmpc']
    variants = {'rti_warm_condensed': {}, 'sqp_warm_condensed': {'solver_type': 'SQP'},
                'rti_cold_condensed': {'warm_start': False},
                'rti_warm_uncondensed': {'condensing_stages': base['horizon']}}
    output.mkdir(parents=True, exist_ok=False)
    manifest = {'schema': 'iet-ch5/solver-replay-v1', 'software': revision(),
                'source_sha256': source_fingerprint(), 'feedback_config_sha256': meta['config_sha256'],
                'feedback_run_id': meta['run_id'], 'parameters': asdict(p), 'repeats': repeats,
                'samples_per_repeat': len(records), 'task': meta['config']['task_config'],
                'scope': 'Measured state/input replay, fixed model/horizon/cost/physical bounds; build/reference preparation excluded. RTI is one SQP step, not an equally converged NLP solution. No realtime-flight or generic solver superiority claim.'}
    summary, samples = {}, {}
    for name, override in variants.items():
        config = {**deepcopy(base), **override}
        calls, internal, statuses, fallbacks, iterations, commands, costs, residuals = [], [], [], [], [], [], [], []
        preparation = []
        for repeat in range(repeats):
            controller = NMPC(config)
            initial = decode(records[0]['truth'])
            controller.reset(initial, task.at(0.), p)
            controller.prepare_reference(task.at, task.duration)
            preparation.append(controller.provenance['reference_preparation_s'])
            # Warm up generated code and the Python interface before the measured replay.
            controller.set_reference_source(task.at, records[0]['t'])
            controller.set_applied_wrench(records[0]['telemetry_estimated_wrench'])
            controller.update(initial, task.at(records[0]['t']), config['design_dt'])
            controller.solution = None
            for row in records:
                state, t = decode(row['truth']), row['t']
                controller.set_reference_source(task.at, t)
                controller.set_applied_wrench(row['telemetry_estimated_wrench'])
                start = time.perf_counter()
                command, diagnostic = controller.update(state, task.at(t), config['design_dt'])
                calls.append(time.perf_counter()-start)
                internal.append(diagnostic['solver_time_s'])
                statuses.append(diagnostic['solver_status_value'])
                fallbacks.append(diagnostic['fallback'])
                iterations.append(diagnostic['solver_iterations'])
                commands.append(command.vector())
                costs.append(controller.solver.get_cost())
                residuals.append(diagnostic['nlp_residuals_stationarity_dynamics_inequality_complementarity'])
        def stats(values):
            return dict(zip(('median', 'p95', 'max'), np.quantile(values, [.5, .95, 1]).tolist()))
        summary[name] = {'config': config, 'controller_call_s': stats(calls), 'solver_internal_s': stats(internal),
                         'fallback_calls': int(sum(fallbacks)), 'calls': len(calls),
                         'status_counts': {str(s): statuses.count(s) for s in sorted(set(statuses))},
                         'sqp_iterations': stats(iterations), 'reference_preparation_s': preparation,
                         'dynamics_residual': stats(np.array(residuals)[:, 1]),
                         'solver_provenance': controller.provenance}
        samples.update({name+'_wall_s': calls, name+'_internal_s': internal, name+'_status': statuses,
                        name+'_fallback': fallbacks, name+'_command': commands, name+'_cost': costs,
                        name+'_residuals': residuals})
        print(name, 'p95 ms', round(summary[name]['controller_call_s']['p95']*1000, 3),
              'fallback', sum(fallbacks), flush=True)
    baseline = np.asarray(samples['sqp_warm_condensed_command'])
    from iet_ch5.allocation import Allocator
    inverse = Allocator(p).inverse
    for name in variants:
        difference = (np.asarray(samples[name+'_command'])-baseline)@inverse.T
        summary[name]['rotor_input_difference_from_sqp'] = {
            'mean_norm': float(np.linalg.norm(difference, axis=1).mean()),
            'max_norm': float(np.linalg.norm(difference, axis=1).max())}
    np.savez_compressed(output/'samples.npz', **samples)
    atomic_json(output/'manifest.json', manifest)
    atomic_json(output/'summary.json', summary)
    lines = ['# acados acceleration: measured-state replay', '', manifest['scope'], '',
             '| Variant | Calls | Controller p95 (ms) | C solver p95 (ms) | Fallbacks |',
             '|---|---:|---:|---:|---:|']
    for name, row in summary.items():
        lines.append(f"| {name} | {row['calls']} | {row['controller_call_s']['p95']*1000:.3f} | {row['solver_internal_s']['p95']*1000:.3f} | {row['fallback_calls']} |")
    lines += ['', 'All variants use the same measured feedback records and reference, but compute different optimization iterates. '
              'RTI speed must be read together with residuals, command differences and closed-loop results. '
              'Cold-start and condensing ablations are implementation options, not independent controller families.']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    plot(output)


def plot(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = json.loads((output/'summary.json').read_text())
    names = list(rows)
    fig, ax = plt.subplots(figsize=(10, 4))
    x = np.arange(len(names))
    ax.bar(x-.18, [rows[n]['controller_call_s']['p95']*1000 for n in names], .36, label='Python + reference + solver + diagnostics')
    ax.bar(x+.18, [rows[n]['solver_internal_s']['p95']*1000 for n in names], .36, label='C solver internal')
    ax.set(xticks=x, xticklabels=[n.replace('_', '\n') for n in names], ylabel='p95 computation time (ms)',
           title='Identical rigid-body OCP / measured-state replay; not equal convergence tolerances')
    ax.legend(fontsize=8)
    fig.tight_layout()
    for ext in ('png', 'svg', 'pdf'):
        fig.savefig(output/f'acceleration.{ext}', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--plot-only', action='store_true')
    args = parser.parse_args()
    if args.plot_only:
        plot(args.output)
    elif args.source is None or args.repeats < 1:
        parser.error('A measured --source run and positive repeats are required')
    else:
        benchmark(args.source, args.output, args.repeats)
