"""Compact measured results and regenerated scientific figures, with failures retained."""

import csv
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from .engine import atomic_json, source_fingerprint
from ..logging import revision

COLORS = dict(zip(('pid', 'lqr', 'geometric', 'dob', 'mpc'),
                  ('#0072B2', '#E69F00', '#009E73', '#CC79A7', '#D55E00')))


def repeatability(rows, data, tolerance):
    groups = {}
    for row in rows:
        key = (row['controller'], row['trajectory'], row['condition'], row['seed'])
        groups.setdefault(key, []).append(row)
    reports = []
    for key, group in groups.items():
        group.sort(key=lambda x: x['repeat'])
        if len(group) < 2:
            continue
        first = data[group[0]['run_id']]
        for other in group[1:]:
            second = data[other['run_id']]
            complete = all(g['metrics'].get('complete_common_interval') for g in (group[0], other))
            same_grid = np.array_equal(first['t'], second['t'])
            delta = float(np.linalg.norm(first['truth_p']-second['truth_p'], axis=1).max()) if same_grid else None
            reports.append({'controller': key[0], 'trajectory': key[1], 'condition': key[2],
                            'seed': key[3], 'repeat': other['repeat'], 'max_position_delta_m': delta,
                            'limit_m': tolerance, 'complete_both': complete,
                            'passed': bool(complete and delta is not None and delta <= tolerance)})
    return reports


def publish(source, destination):
    source, destination = Path(source), Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    protocol = json.loads((source/'protocol.json').read_text())
    atomic_json(destination/'analysis_provenance.json', {'software': revision(),
                'source_sha256': source_fingerprint(), 'role': 'postprocessing only; raw run metadata/protocol preserved'})
    journal = json.loads((source/'progress.json').read_text())
    rows = list(journal.values())
    if any(row['status'] in ('running', 'interrupted', 'infrastructure_failed') for row in rows):
        raise ValueError('Resolve incomplete infrastructure attempts before publication')
    expected = len(protocol['resolved']['matrix'])
    if len(rows) != expected:
        raise ValueError('Experiment matrix incomplete')
    for name in ('protocol.json', 'preflight.json', 'progress.json'):
        shutil.copy2(source/name, destination/name)
    data = {}
    hashes = {}
    for row in rows:
        key = row['run_id']
        src, dest = source/'runs'/key, destination/'runs'/key
        dest.mkdir(parents=True)
        for name in ('metadata.json', 'result.json', 'evaluation.json', 'metrics_samples.npz',
                     'failure.txt', 'camera_provenance.json', 'frames.json'):
            if (src/name).exists():
                shutil.copy2(src/name, dest/name)
        data[key] = dict(np.load(src/'metrics_samples.npz'))
        for name in ('log.jsonl', 'physics.jsonl'):
            path = src/name
            hashes[f'{key}/{name}'] = {'bytes': path.stat().st_size,
                'sha256': hashlib.file_digest(path.open('rb'), 'sha256').hexdigest()} if hasattr(hashlib, 'file_digest') else {
                    'bytes': path.stat().st_size, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    atomic_json(destination/'raw_log_inventory.json', hashes)
    atomic_json(destination/'summary.json', rows)
    repeat = repeatability(rows, data, protocol['resolved']['suite'].get('repeatability_max_position_delta_m', .02))
    atomic_json(destination/'repeatability.json', repeat)
    fields = ['run_id', 'controller', 'trajectory', 'tier', 'condition', 'seed', 'repeat', 'status',
              'complete_common_interval', 'tracking_rms_m', 'position_max_m', 'velocity_rms_m_s',
              'heading_rms_rad', 'constraint_violation_duration_s', 'fallback_count', 'failure']
    with (destination/'summary.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            flat = {**row, **row['metrics']}
            writer.writerow({k: flat.get(k) for k in fields})
    plot(rows, data, destination/'figures')
    counts = {s: sum(r['status'] == s for r in rows) for s in sorted({r['status'] for r in rows})}
    lines = ['# Modular UAV case study', '', f'Measured AirSim runs: {len(rows)}. Status counts: `{counts}`.', '',
             f"Source commit: `{protocol['software']['commit']}`; executable/config content SHA256: `{protocol['source_sha256']}`.", '',
             'All controllers use the same verified motor allocation and independent physics evaluation. '
             'Paused simulation timing does not establish real-time feasibility. Controller-owned defaults are frozen in protocol.json. '
             'Aggressive flight is evaluated separately from the near-hover design envelope; no claim of statistical superiority.', '',
             '| Trajectory | Controller | Pass / runs | Tracking RMS range (m), complete runs |',
             '|---|---|---:|---:|']
    for trajectory in protocol['resolved']['trajectories']:
        for controller in protocol['resolved']['suite']['controllers']:
            group = [r for r in rows if r['trajectory'] == trajectory['id'] and r['controller'] == controller]
            values = [r['metrics']['tracking_rms_m'] for r in group if r['metrics'].get('complete_common_interval')]
            span = f'{min(values):.4f}–{max(values):.4f}' if values else 'censored / incomplete'
            lines.append(f"| {trajectory['id']} ({trajectory['tier']}) | {controller} | {sum(r['status']=='passed' for r in group)} / {len(group)} | {span} |")
    lines += ['', f'Repeatability checks: {sum(r["passed"] for r in repeat)} / {len(repeat)} within the predeclared position-delta tolerance. '
              'Two repeats are a reproducibility check, not a powered statistical comparison.', '',
              'Aborted runs retain their available-interval metrics and failure reason, but are excluded from complete-interval RMS comparisons. '
              'All failed complete runs remain in tables/plots. Raw control/physics logs remain in the ignored local archive; '
              'raw_log_inventory.json records their individual hashes. External storage upload is not claimed.', '',
              'Figures can be regenerated from the committed NPZ files with `scripts/plot_case_study.py`. '
              'Force-pulse injection remains unavailable; wind, mismatch and paired noise/delay protocols are configured separately.']
    (destination/'REPORT.md').write_text('\n'.join(lines)+'\n')
    if protocol['capture_demo']:
        from .media import render
        render(source, destination, rows, data)
    print(counts)


def plot(rows, data, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    font = 'Times New Roman' if 'Times New Roman' in {f.name for f in font_manager.fontManager.ttflist} else 'DejaVu Serif'
    plt.rcParams.update({'font.family': font, 'font.size': 9, 'axes.grid': True, 'grid.alpha': .2,
                         'svg.fonttype': 'none', 'pdf.fonttype': 42})
    def save(fig, name):
        fig.tight_layout()
        for ext in ('png', 'pdf', 'svg'):
            fig.savefig(output/f'{name}.{ext}', dpi=160)
        plt.close(fig)
    for trajectory in sorted({r['trajectory'] for r in rows}):
        group = [r for r in rows if r['trajectory'] == trajectory and r['repeat'] == 0]
        fig = plt.figure(figsize=(13, 4))
        axes = [fig.add_subplot(131, projection='3d'), fig.add_subplot(132), fig.add_subplot(133)]
        first = data[group[0]['run_id']]
        axes[0].plot(*first['reference_p'].T, 'k--', label='Reference')
        axes[1].plot(*first['reference_p'][:, :2].T, 'k--', label='Reference')
        for row in group:
            d, m = data[row['run_id']], row['controller']
            label = f"{m} ({row['status']})"
            axes[0].plot(*d['truth_p'].T, color=COLORS[m], label=label)
            axes[1].plot(*d['truth_p'][:, :2].T, color=COLORS[m])
            axes[2].plot(d['t'], np.linalg.norm(d['truth_p']-d['reference_p'], axis=1), color=COLORS[m], label=label)
        axes[0].set(xlabel='North (m)', ylabel='West (m)', zlabel='Up (m)', title=trajectory)
        axes[1].set(xlabel='North (m)', ylabel='West (m)', aspect='equal', title='Top-down trajectory')
        axes[2].set(xlabel='Simulation time (s)', ylabel='Position error norm (m)')
        axes[2].legend(fontsize=6)
        save(fig, trajectory+'_tracking')
        fig, axes = plt.subplots(2, 2, figsize=(11, 7))
        for row in group:
            d, m = data[row['run_id']], row['controller']
            color = COLORS[m]
            axes[0, 0].plot(d['t'], d['requested_wrench'][:, 0], color=color, label=m)
            axes[0, 0].plot(d['t'], d['telemetry_estimated_wrench'][:, 0], '--', color=color, alpha=.5)
            axes[0, 1].plot(d['t'], np.linalg.norm(d['requested_wrench'][:, 1:], axis=1), color=color)
            axes[0, 1].plot(d['t'], np.linalg.norm(d['telemetry_estimated_wrench'][:, 1:], axis=1), '--', color=color, alpha=.5)
            axes[1, 0].plot(d['t'], d['motor_pwm'].min(axis=1), color=color, alpha=.7)
            axes[1, 0].plot(d['t'], d['motor_pwm'].max(axis=1), color=color, label=m)
            axes[1, 1].plot(d['t'], np.linalg.norm(d['truth_v'], axis=1), color=color, label=m)
        for ax in axes.flat:
            ax.set_xlabel('Simulation time (s)')
        axes[0, 0].set(ylabel='Collective (N)', title='Solid: requested; dashed: telemetry estimate')
        axes[0, 1].set(ylabel='Moment norm (N m)', title='Requested / telemetry-estimated moments')
        axes[1, 0].set(ylabel='Normalized motor input range', ylim=(-.02, 1.02))
        axes[1, 1].set(ylabel='Actual speed (m/s)')
        axes[0, 0].legend(ncol=3, fontsize=7)
        save(fig, trajectory+'_inputs')
    methods = list(COLORS)
    paths = sorted({r['trajectory'] for r in rows})
    fig, ax = plt.subplots(figsize=(11, 5))
    matrix = np.full((len(methods), len(paths)), np.nan)
    complete_rms = [r['metrics'].get('tracking_rms_m') or 0. for r in rows
                    if r['metrics'].get('complete_common_interval')]
    colour_max = max(.15, max(complete_rms, default=0.))
    for i, m in enumerate(methods):
        for j, p in enumerate(paths):
            group = [r for r in rows if r['controller'] == m and r['trajectory'] == p]
            full = [r['metrics']['tracking_rms_m'] for r in group if r['metrics'].get('complete_common_interval')]
            if full:
                matrix[i, j] = np.mean(full)
            text = f'{np.mean(full):.3f}' if full else 'incomplete'
            text += f"\n{sum(r['status']=='passed' for r in group)}/{len(group)} pass"
            ax.text(j, i, text, ha='center', va='center', fontsize=8,
                    color='white' if full and np.mean(full) > .65*colour_max else '#111111')
    im = ax.imshow(np.ma.masked_invalid(matrix), cmap='YlOrRd', aspect='auto', vmin=0, vmax=colour_max)
    ax.set(xticks=range(len(paths)), xticklabels=paths, yticks=range(len(methods)), yticklabels=methods,
           title='Tracking RMS (m), complete runs; all failures retained in pass counts')
    ax.tick_params(axis='x', labelrotation=25)
    ax.grid(False)
    fig.colorbar(im, ax=ax, label='Mean complete-run tracking RMS (m)')
    save(fig, 'accuracy_matrix')
    fig, ax = plt.subplots(figsize=(8, 4))
    for i, m in enumerate(methods):
        group = [r for r in rows if r['controller'] == m]
        values = [r['metrics']['controller_s']['p95']*1000 for r in group
                  if 'controller_s' in r['metrics']]
        ax.scatter(np.full(len(values), i), values, color=COLORS[m], alpha=.5)
    ax.set(xticks=range(len(methods)), xticklabels=methods, ylabel='Per-run controller p95 (ms)',
           title='Paused benchmark computation; not a real-time demonstration')
    save(fig, 'timing')
