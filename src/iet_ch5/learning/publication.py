"""All-pair compact evidence and book figures; regenerate without a simulator."""
import json
from pathlib import Path
import shutil

import numpy as np

from ..experiments.engine import atomic_json
from ..logging import revision
from .gp import sha256
from .workflow import matched_pair_configuration


def publish(source, dataset, models, validation, output):
    source, output = Path(source), Path(output)
    protocol = json.loads((source/'protocol.json').read_text())
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(source/'protocol.json', output/'protocol.json')
    rows, raw_files = [], {}
    for pair in protocol['pairs']:
        pair_metadata = {}
        for variant in ('off','on'):
            entry = pair['runs'][variant]
            run = source/entry['group']/'runs'/entry['run_id']
            target = output/'runs'/entry['run_id']
            target.mkdir(parents=True)
            for name in ('metadata.json','result.json','evaluation.json','metrics_samples.npz','failure.txt'):
                if (run/name).exists():
                    shutil.copy2(run/name,target/name)
            meta = json.loads((run/'metadata.json').read_text())
            pair_metadata[variant] = meta
            controls = [json.loads(line) for line in (run/'log.jsonl').read_text().splitlines()]
            solves = [r for r in controls if r['diagnostics'].get('qp_solved_this_update')]
            update = np.array([r['timing']['controller_s'] for r in solves])
            native = np.array([r['diagnostics']['solver_time_s'] for r in solves])
            np.savez_compressed(target/'timing_samples.npz', update_s=update, native_s=native,
                t=np.array([r['t'] for r in solves]),
                solver_status=np.array([r['diagnostics']['solver_status_value'] for r in solves]),
                fallback=np.array([r['diagnostics']['fallback'] for r in solves]))
            row = {k:pair[k] for k in ('condition','trajectory','repeat')}
            row.update(entry['evaluation'])
            row.update(variant=variant,run_id=entry['run_id'])
            row.update(update_p95_ms=float(np.quantile(update,.95)*1e3),
                       update_max_ms=float(np.max(update)*1e3),native_p95_ms=float(np.quantile(native,.95)*1e3))
            rows.append(row)
            for name in ('log.jsonl','physics.jsonl'):
                raw_files[entry['run_id']+'/'+name] = {'bytes':(run/name).stat().st_size,'sha256':sha256(run/name)}
        off, on = pair_metadata['off'], pair_metadata['on']
        matched_pair_configuration(off['config']['controllers'],on['config']['controllers'])
        for key in ('parameters','controller_parameters'):
            if off[key] != on[key]:
                raise ValueError('Pair physical/controller parameters differ')
        if off['config']['task_config'] != on['config']['task_config']:
            raise ValueError('Pair reference/condition differs')
        for key in ('p','v','R','omega'):
            if not np.allclose(off['verified_initial_state'][key],on['verified_initial_state'][key],atol=1e-9,rtol=0):
                raise ValueError('Pair initial state differs')
    atomic_json(output/'summary.json',rows)
    atomic_json(output/'raw_log_inventory.json',raw_files)
    atomic_json(output/'analysis_provenance.json',{'software':revision(),
        'dataset_manifest_sha256':sha256(Path(dataset)/'manifest.json'),
        'models_manifest_sha256':sha256(Path(models)/'manifest.json'),
        'validation_report_sha256':sha256(Path(validation)/'REPORT.json'),
        'pair_checks':'initial p/v/R/omega, physical/model parameters, task and all controller settings except residual.enabled verified'})
    repeats = []
    for condition in sorted({r['condition'] for r in rows}):
        for trajectory in sorted({r['trajectory'] for r in rows}):
            for variant in ('off','on'):
                group = sorted([r for r in rows if (r['condition'],r['trajectory'],r['variant']) == (condition,trajectory,variant)],key=lambda r:r['repeat'])
                for other in group[1:]:
                    a = dict(np.load(output/'runs'/group[0]['run_id']/'metrics_samples.npz'))
                    b = dict(np.load(output/'runs'/other['run_id']/'metrics_samples.npz'))
                    grid = a['t']
                    pb = np.column_stack([np.interp(grid,b['t'],b['truth_p'][:,i]) for i in range(3)])
                    complete = all(r['metrics']['complete_common_interval'] for r in (group[0],other))
                    delta = float(np.max(np.linalg.norm(a['truth_p']-pb,axis=1)))
                    repeats.append({'condition':condition,'trajectory':trajectory,'variant':variant,
                        'repeat':other['repeat'],'max_position_delta_m':delta,
                        'passed':bool(complete and delta <= protocol['suite']['repeatability_max_position_delta_m'])})
    atomic_json(output/'repeatability.json',repeats)
    lines = ['# Matched GP force-residual NMPC: measured AirSim results','',
        f"Numerical freeze: `{protocol['software']['commit']}`; source/config SHA256 `{protocol['source_sha256']}`.",'',
        'Same rigid-body NMPC with a frozen GP force residual disabled/enabled. All pairs have verified equal initial states, physical/model parameters, trajectory, horizon, cost, constraints, feedback and actuator settings. The sole controller-setting difference is `residual.enabled`.', '',
        '| Condition | Trajectory | Repeat | GP | Status | Traversal RMS (mm) | Peak (mm) | Fallbacks | Violations (s) | Update p95 (ms) |',
        '|---|---|---:|---|---|---:|---:|---:|---:|---:|']
    for r in rows:
        m=r['metrics']
        lines.append(f"| {r['condition']} | {r['trajectory']} | {r['repeat']} | {r['variant']} | {r['status']} | {m['tracking_rms_m']*1000:.3f} | {m['position_max_m']*1000:.3f} | {m['fallback_count']} | {m['constraint_violation_duration_s']:.4f} | {r['update_p95_ms']:.3f} |")
    lines+=['',f"Repeatability: {sum(r['passed'] for r in repeats)}/{len(repeats)} within the predeclared 20 mm maximum trajectory-difference limit.",'',
        'Every attempted closed-loop run and failed outcome is retained. Traversal windows come from the unchanged trajectory presets; full-duration peak and constraint statistics include initial/final holds. Two repeats check reproducibility, not statistical superiority. Paused stepping, computation spikes and native-build preparation preclude a real-time claim. Learned force is a model-discrepancy estimate, not measured wind. Nominal and mass-bias models are trained separately; neither is presented as an online adaptive or universal model. Raw logs remain in the identified local archive; compact truth/configuration/timing arrays are committed.']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    plot(output,output/'figures')
    return rows


def plot(source, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    font = 'Times New Roman' if 'Times New Roman' in {f.name for f in font_manager.fontManager.ttflist} else 'DejaVu Serif'
    plt.rcParams.update({'font.family':font,'font.size':9,'axes.grid':True,'grid.alpha':.2,'svg.fonttype':'none','pdf.fonttype':42})
    rows = json.loads((source/'summary.json').read_text())
    colors = {'off':'#D55E00','on':'#0072B2'}
    def save(fig,name):
        fig.tight_layout()
        for ext in ('png','svg','pdf'):
            fig.savefig(output/(name+'.'+ext),dpi=160)
        plt.close(fig)
    for condition in sorted({r['condition'] for r in rows}):
        for trajectory in sorted({r['trajectory'] for r in rows}):
            group = [r for r in rows if r['condition'] == condition and r['trajectory'] == trajectory]
            fig=plt.figure(figsize=(13,4))
            axes=[fig.add_subplot(131,projection='3d'),fig.add_subplot(132),fig.add_subplot(133)]
            for r in group:
                d=dict(np.load(source/'runs'/r['run_id']/'metrics_samples.npz'))
                variant=r['variant']
                if r['repeat']==0:
                    axes[0].plot(*d['truth_p'].T,color=colors[variant],label='GP '+variant)
                    axes[1].plot(*d['truth_p'][:,:2].T,color=colors[variant])
                axes[2].plot(d['t'],np.linalg.norm(d['truth_p']-d['reference_p'],axis=1)*1000,
                             color=colors[variant],ls='-' if r['repeat']==0 else '--',label=f"GP {variant}, repeat {r['repeat']}")
            axes[0].plot(*d['reference_p'].T,'k:',label='Reference')
            axes[1].plot(*d['reference_p'][:,:2].T,'k:')
            axes[0].set(xlabel='North (m)',ylabel='West (m)',zlabel='Up (m)',title=condition+' / '+trajectory)
            axes[0].legend(fontsize=7)
            axes[1].set(xlabel='North (m)',ylabel='West (m)',aspect='equal',title='Top-down path')
            axes[2].set(xlabel='Simulation time (s)',ylabel='Position error (mm)',title='All repeats, including holds')
            axes[2].legend(fontsize=7)
            save(fig,condition+'_'+trajectory+'_tracking')
            fig,axes=plt.subplots(2,2,figsize=(11,6))
            for r in group:
                d=dict(np.load(source/'runs'/r['run_id']/'metrics_samples.npz'))
                color=colors[r['variant']]
                style='-' if r['repeat']==0 else '--'
                label=f"GP {r['variant']}, r{r['repeat']}"
                axes[0,0].plot(d['t'],d['requested_wrench'][:,0],color=color,ls=style,label=label)
                axes[0,1].plot(d['t'],d['telemetry_estimated_wrench'][:,0],color=color,ls=style)
                axes[1,0].plot(d['t'],d['motor_pwm'].min(axis=1),color=color,ls=style)
                axes[1,0].plot(d['t'],d['motor_pwm'].max(axis=1),color=color,ls=style,alpha=.6)
                axes[1,1].plot(d['t'],np.linalg.norm(d['requested_wrench'][:,1:],axis=1),color=color,ls=style)
            axes[0,0].set(ylabel='Requested collective (N)',title=condition+' / '+trajectory)
            axes[0,1].set(ylabel='Telemetry-estimated applied collective (N)')
            axes[1,0].set(ylabel='Motor input envelope [0,1]')
            axes[1,1].set(ylabel='Requested body-moment norm (N m)')
            for ax in axes.flat:
                ax.set_xlabel('Simulation time (s)')
            axes[0,0].legend(fontsize=7)
            save(fig,condition+'_'+trajectory+'_inputs')
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    for variant,color in colors.items():
        group=[r for r in rows if r['variant']==variant]
        index=np.arange(len(group))+(0 if variant=='off' else .35)
        axes[0].bar(index,[r['metrics']['tracking_rms_m']*1000 for r in group],width=.35,color=color,label='GP '+variant)
        axes[1].bar(index,[r['update_p95_ms'] for r in group],width=.35,color=color,label='GP '+variant)
    labels=[r['condition']+'\n'+r['trajectory']+'\nr'+str(r['repeat']) for r in group]
    for ax in axes:
        ax.set_xticks(np.arange(len(group))+.175,labels,rotation=40,ha='right',fontsize=7)
        ax.legend()
    axes[0].set(ylabel='Traversal RMS (mm)',yscale='log',title='Every paired run')
    axes[1].set(ylabel='Controller update p95 (ms)',title='Paused simulation; wall time')
    save(fig,'accuracy_and_timing')


def plot_prediction(source, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    source, output = Path(source), Path(output)
    output.mkdir(parents=True,exist_ok=True)
    from matplotlib import font_manager
    font = 'Times New Roman' if 'Times New Roman' in {f.name for f in font_manager.fontManager.ttflist} else 'DejaVu Serif'
    plt.rcParams.update({'font.family':font,'font.size':9,'axes.grid':True,'grid.alpha':.2,'svg.fonttype':'none','pdf.fonttype':42})
    report=json.loads((source/'REPORT.json').read_text())
    for quantity,label in ((0,'Position prediction RMSE (m)'),(1,'Velocity prediction RMSE (m/s)')):
        fig,axes=plt.subplots(2,2,figsize=(10,6))
        for row,ax in zip(report['rows'],axes.flat):
            data=dict(np.load(source/(row['flight_id']+'.npz')))
            for i,variant in enumerate(('nominal','GP residual')):
                ax.plot(data['horizon_t'],data['error_curve'][i,:,quantity],label=variant)
            ax.set(xlabel='Open-loop prediction horizon (s)',ylabel=label,
                   title=row['condition']+' / '+row['trajectory'])
            ax.legend(fontsize=7)
        fig.tight_layout()
        for ext in ('png','svg','pdf'):
            fig.savefig(output/('prediction_'+('position' if quantity==0 else 'velocity')+'.'+ext),dpi=160)
        plt.close(fig)
