"""Compare the original and retimed reference, and independently measured flight speeds."""

import argparse
import json
import shutil
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

from iet_ch5.evaluation.reanalyze import reanalyze
from iet_ch5.logging import dump, revision
from iet_ch5.references.tasks import Task

COLORS = dict(zip(['pid', 'lqr', 'geometric', 'dob', 'mpc'],
                 ['#0072B2', '#E69F00', '#009E73', '#CC79A7', '#D55E00']))


def plot(previous, current):
    baseline = current / 'previous_speed_samples'
    baseline.mkdir(exist_ok=False)
    table = []
    now = {r['controller']: r for r in json.loads((current/'summary.json').read_text())}
    for method in COLORS:
        out = baseline / method
        out.mkdir()
        metrics, _ = reanalyze(previous/method, out/'metric_samples.npz')
        for name in ['metadata.json', 'result.json']:
            shutil.copy2(previous/method/name, out/name)
        (out/'metrics_common_interval.json').write_text(dump(metrics)+'\n')
        new = now[method]['metrics']
        table.append({'controller': method,
            'old_mean_m_s': metrics['truth_traversal_mean_speed_m_s'],
            'new_mean_m_s': new['truth_traversal_mean_speed_m_s'],
            'old_peak_m_s': metrics['truth_traversal_peak_speed_m_s'],
            'new_peak_m_s': new['truth_traversal_peak_speed_m_s'],
            'mean_speed_ratio': new['truth_traversal_mean_speed_m_s']/metrics['truth_traversal_mean_speed_m_s']})
    old_c = json.loads((previous/'manifest.json').read_text())['task_config']
    new_c = json.loads((current/'capture_manifest.json').read_text())['task_config']
    old, new = Task('figure_eight', old_c), Task('figure_eight', new_c)
    font = ('Times New Roman' if 'Times New Roman' in
            {f.name for f in font_manager.fontManager.ttflist} else 'DejaVu Serif')
    plt.rcParams.update({'font.family': font, 'font.size': 10,
                         'axes.grid': True, 'grid.alpha': .2, 'svg.fonttype': 'none', 'pdf.fonttype': 42})
    fig, axes = plt.subplots(2, 1, figsize=(9, 7))
    reference_stats = []
    for task, style, label in [(old, '--', 'Previous: 28 s traversal'),
                                (new, '-', 'Retimed: 14 s traversal')]:
        t = np.linspace(0, task.duration, 4001)
        speed = np.array([np.linalg.norm(task.at(float(x)).v) for x in t])
        axes[0].plot(t, speed, style, color='gray' if task is old else 'black', label=label)
        a, b = task.tracking_interval
        ts = np.linspace(a, b, 4001)
        vs = np.array([np.linalg.norm(task.at(float(x)).v) for x in ts])
        reference_stats.append({'traversal_s': b-a,
            'mean_speed_m_s': float(np.trapezoid(vs, ts)/(b-a)),
            'peak_speed_m_s': float(vs.max()),
            'path_length_m': float(np.trapezoid(vs, ts))})
    onset, end = new.tracking_interval
    ramp = new_c['figure_eight_ramp_s']
    axes[0].axvspan(onset+ramp, end-ramp, color='orange', alpha=.12, label='Sustained cruise interval')
    axes[0].set(xlabel='Simulation time (s)', ylabel='Reference speed (m/s)',
                title='Same geometric path; shorter actual timing')
    axes[0].legend(fontsize=9)
    for method in COLORS:
        d = np.load(current/method/'metric_samples.npz')
        axes[1].plot(d['t'], np.linalg.norm(d['truth_v'],axis=1), color=COLORS[method], label=method.upper())
    t = np.linspace(0, new.duration, 2001)
    axes[1].plot(t, [np.linalg.norm(new.at(float(x)).v) for x in t], 'k--', alpha=.6, label='Reference')
    axes[1].axvspan(onset+ramp, end-ramp, color='orange', alpha=.12)
    axes[1].set(xlabel='Simulation time (s)', ylabel='Measured speed norm (m/s)',
                title='New runs: independent AirSim physics truth')
    axes[1].legend(ncol=3, fontsize=9)
    fig.tight_layout()
    for ext in ['png','pdf','svg']:
        fig.savefig(current/f'speed_comparison.{ext}', dpi=180)
    plt.close(fig)
    report={'reference_statistics':reference_stats,'actual_speed_comparison':table,
            'old_run_source':'figure-eight-feasible-20261020',
            'metric_method':'Independent physics velocities on the common 12 ms grid; mean of speed norm over each actual traversal window',
            'analysis_software':revision(),'source':'scripts/plot_speed_comparison.py','font':font}
    (current/'speed_comparison.json').write_text(dump(report)+'\n')
    print(dump(report))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--previous',type=Path,required=True)
    p.add_argument('--current',type=Path,required=True)
    a=p.parse_args()
    plot(a.previous,a.current)
