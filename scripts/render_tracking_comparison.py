"""Publish synchronized actual AirSim scenes and plots from a common-reference capture."""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import imageio.v2 as imageio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from PIL import Image
from scipy.optimize import brentq

from iet_ch5.evaluation.reanalyze import reanalyze
from iet_ch5.logging import dump, revision
from iet_ch5.references.tasks import Task

METHODS = ['pid', 'lqr', 'geometric', 'dob', 'mpc']
NAMES = {'pid': 'Cascaded PID', 'lqr': 'LQR', 'geometric': 'Geometric SE(3)',
         'dob': 'Geometric + force DOB', 'mpc': 'Constrained linear MPC'}
COLORS = dict(zip(METHODS, ['#0072B2', '#E69F00', '#009E73', '#CC79A7', '#D55E00']))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()



def save_compact_gif(frames, path):
    # A fixed palette reduces scene noise while reserving every controller/marker colour.
    # This affects only the preview; original PNGs and the MP4 retain full colour.
    values = frames[len(frames)//2].quantize(
        colors=64, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE).getpalette()[:192]
    for colour in list(COLORS.values()) + ['#FFFF80', '#FFFF00', '#00FFFF',
                                          '#101923', '#F6F8FA', '#FFFFFF']:
        values.extend(int(colour[i:i+2], 16) for i in [1, 3, 5])
    values += values[-3:] * ((768-len(values))//3)
    palette = Image.new('P', (1, 1))
    palette.putpalette(values)
    indexed = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]
    indexed[0].save(path, save_all=True, append_images=indexed[1:],
                    duration=100, loop=0, optimize=True)


def publish(source, output):
    output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((source / 'manifest.json').read_text())
    shutil.copy2(source / 'manifest.json', output / 'capture_manifest.json')
    configs, samples, frame_sets, summary = [], {}, {}, []
    fixed_pose = None
    native_frame_count = 0
    for method in METHODS:
        run_dir = source / method
        dest = output / method
        dest.mkdir()
        for filename in ['metadata.json', 'result.json', 'camera_provenance.json', 'frames.json']:
            shutil.copy2(run_dir / filename, dest / filename)
        metrics, data = reanalyze(run_dir, dest / 'metric_samples.npz')
        if not metrics['complete_common_interval']:
            raise RuntimeError(f'{method} did not complete the common interval')
        samples[method] = data
        (dest / 'metrics_common_interval.json').write_text(dump(metrics) + '\n')
        result = json.loads((run_dir / 'result.json').read_text())
        summary.append({'controller': method, 'status': result['status'],
                        'metrics': metrics, 'runtime_metrics': result['metrics']})
        meta = json.loads((run_dir / 'metadata.json').read_text())
        configs.append(meta['config']['task_config'])
        frames = json.loads((run_dir / 'frames.json').read_text())
        native_frame_count += len(frames)
        for frame in frames:
            pose = frame['camera_info']['pose']
            values = [pose['position'][k + '_val'] for k in 'xyz'] + [
                pose['orientation'][k + '_val'] for k in 'xyzw']
            if fixed_pose is None:
                fixed_pose = values
            if not np.allclose(values, fixed_pose, atol=1e-5, rtol=0.):
                raise RuntimeError('Fixed camera poses differ across the comparison')
        times = np.array([f['simulation_t'] for f in frames])
        if not np.all(np.diff(times) > 0):
            raise RuntimeError('Nonmonotonic image acquisition times')
        for f in frames:
            if sha(run_dir / 'frames' / f['filename']) != f['sha256']:
                raise RuntimeError('Frame checksum mismatch')
            if f['image_timestamp_ns'] != f['truth_timestamp_ns']:
                raise RuntimeError('Image capture was not at the paused physics truth timestamp')
        frame_sets[method] = (frames, times)
    if not all(c == configs[0] for c in configs):
        raise RuntimeError('Controller references differ')
    for method in METHODS[1:]:
        if not np.array_equal(samples[method]['reference_p'], samples['pid']['reference_p']):
            raise RuntimeError('Common sampled reference differs')
    cfg = configs[0]
    task_name = manifest['task']
    task = Task(task_name, cfg)
    duration = task.duration
    onset, end = task.tracking_interval
    crossings = []
    if task_name == 'figure_eight':
        path_label = 'FIGURE EIGHT'
        crossings = [brentq(lambda t: task.phase_at(t)[0] - phase, onset, end)
                     for phase in [np.pi/2, 3*np.pi/2]]
        extent = f"{2*cfg['figure_eight_half_width_m']:g} x {2*cfg['figure_eight_half_height_m']:g} m"
        trajectory_title = f'Same predefined figure eight ({extent})'
    else:
        path_label = 'CIRCLE'
        extent = f"diameter {2*cfg['circle_radius']:g} m"
        trajectory_title = f'Same predefined circle ({extent})'
    t = samples['pid']['t']
    reference = samples['pid']['reference_p']
    error = {m: np.linalg.norm(samples[m]['truth_p'] - samples[m]['reference_p'], axis=1)
             for m in METHODS}
    (output / 'summary.json').write_text(dump(summary) + '\n')
    font = ('Times New Roman' if 'Times New Roman' in
            {f.name for f in font_manager.fontManager.ttflist} else 'DejaVu Serif')
    plt.rcParams.update({'font.family': font, 'font.size': 10, 'axes.grid': True,
                         'grid.alpha': .2, 'svg.fonttype': 'none', 'pdf.fonttype': 42})

    # Static book figures, with metres on trajectories and millimetres on errors.
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    axes[0].plot(*reference[:, :2].T, 'k--', lw=2, label='Reference')
    for m in METHODS:
        p = samples[m]['truth_p']
        axes[0].plot(*p[:, :2].T, color=COLORS[m], label=NAMES[m])
        axes[1].plot(t, error[m] * 1000, color=COLORS[m], label=NAMES[m])
    axes[0].set(xlabel='North x (m)', ylabel='West y (m)', aspect='equal',
                title=trajectory_title + '\nBlack dashed: reference')
    axes[1].set(xlabel='Simulation time (s)', ylabel='Position error norm (mm)',
                title='Independent AirSim physics truth')
    axes[1].axvspan(onset, end, color='gray', alpha=.12)
    for crossing in crossings:
        axes[1].axvline(crossing, color='gray', linestyle=':', alpha=.6)
    if crossings:
        axes[1].set_title('Independent AirSim physics truth\nDotted: centre crossings')
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    for ext in ['png', 'pdf', 'svg']:
        fig.savefig(output / f'trajectories_and_errors.{ext}', dpi=180)
    plt.close(fig)

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10})
    fig = plt.figure(figsize=(16, 10.08), dpi=100, facecolor='#101923')
    grid = fig.add_gridspec(2, 3, left=.015, right=.985, bottom=.065, top=.86,
                           wspace=.06, hspace=.14)
    image_axes = [fig.add_subplot(grid[i // 3, i % 3]) for i in range(5)]
    images, captions = {}, {}
    for m, ax in zip(METHODS, image_axes):
        ax.set_axis_off()
        images[m] = ax.imshow(imageio.imread(source / m / 'frames' / '00000.png'))
        captions[m] = ax.set_title(NAMES[m], color=COLORS[m], fontsize=13, fontweight='bold')
    ax = fig.add_subplot(grid[1, 2], facecolor='#f6f8fa')
    lines = {}
    for m in METHODS:
        ax.plot(t, error[m] * 1000, color=COLORS[m], alpha=.16, linewidth=1)
        lines[m], = ax.plot([], [], color=COLORS[m], label=m.upper(), linewidth=2)
    for crossing in crossings:
        ax.axvline(crossing, color='gray', linestyle=':', alpha=.4)
    cursor = ax.axvline(0, color='#4b5563', linestyle='--', linewidth=1)
    ax.set(xlim=(0, duration), ylim=(0, max(e.max() for e in error.values()) * 1100 + 1),
           xlabel='Simulation time (s)', ylabel='Position error norm (mm)')
    ax.tick_params(colors='white')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.legend(loc='upper left', ncol=3, fontsize=8)
    ax.set_title('Tracking error' + (' | dotted: centre crossings' if crossings else ''),
                 color='white', fontsize=12)
    heading = fig.text(.025, .967, '', color='white', fontsize=19, fontweight='bold')
    fig.text(.025, .929,
             f"ONE REFERENCE: {path_label}  |  {extent}  |  traversal {end-onset:g} s  |  local altitude {cfg['initial_position'][2]:g} m  |  fixed heading",
             color='#cbd5e1', fontsize=11)
    fig.text(.025, .02,
             'Actual AirSim frames  |  yellow: reference  |  cyan: flown trace  |  fixed external camera'
             '  |  paused simulation; MP4 1x / GIF 2x', color='#cbd5e1', fontsize=10)
    video_times = np.arange(0., duration, .1)
    timeline = []
    gif_frames = []
    poster_time = round((onset + end) / 2, 1)
    with imageio.get_writer(output / 'comparison.mp4', fps=10, codec='libx264', quality=8,
                            macro_block_size=16) as writer:
        for video_t in video_times:
            phase = 'HOLD' if video_t < onset or video_t >= end else f'TRACK {path_label}'
            if cfg.get('figure_eight_timing') == 'cruise' and onset <= video_t < end:
                ramp = cfg['figure_eight_ramp_s']
                phase = ('ACCELERATE' if video_t < onset+ramp else
                         'DECELERATE' if video_t >= end-ramp else 'CRUISE')
            heading.set_text(f'Five controllers / one trajectory     t = {video_t:04.1f} s     {phase}')
            record = {'video_t_s': float(video_t), 'scene_frames': {}}
            for m in METHODS:
                frames, times = frame_sets[m]
                index = max(0, np.searchsorted(times, video_t + 1e-9, side='right') - 1)
                f = frames[index]
                age = float(video_t - f['simulation_t'])
                if age < -1e-8 or age > .15:
                    raise RuntimeError('Scene/truth synchronization exceeds 150 ms')
                images[m].set_data(imageio.imread(source / m / 'frames' / f['filename']))
                frame_error = np.linalg.norm(np.array(f['truth_p_nwu_m']) - f['reference_p_nwu_m'])
                speed_label = (f'  |  v {np.linalg.norm(f["truth_v_nwu_m_s"]):.2f} m/s'
                               if 'truth_v_nwu_m_s' in f else '')
                captions[m].set_text(f'{NAMES[m]}\nframe {f["simulation_t"]:.2f} s  |  err {frame_error*1000:.2f} mm' + speed_label)
                mask = t <= video_t
                lines[m].set_data(t[mask], error[m][mask] * 1000)
                record['scene_frames'][m] = {'filename': f['filename'],
                    'simulation_t_s': f['simulation_t'], 'age_s': age, 'sha256': f['sha256']}
            cursor.set_xdata([video_t, video_t])
            fig.canvas.draw()
            rgb = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
            writer.append_data(rgb)
            if len(timeline) % 2 == 0:
                gif_frames.append(Image.fromarray(rgb).resize((960, 600), Image.Resampling.LANCZOS))
            if abs(video_t - poster_time) < .01:
                imageio.imwrite(output / 'comparison_preview.png', rgb)
                for m in METHODS:
                    f = record['scene_frames'][m]
                    shutil.copy2(source / m / 'frames' / f['filename'], output / m / 'scene.png')
            timeline.append(record)
    plt.close(fig)
    save_compact_gif(gif_frames, output / 'comparison.gif')
    (output / 'video_timeline.json').write_text(dump(timeline) + '\n')
    reader = imageio.get_reader(output / 'comparison.mp4')
    count = 0
    sizes = set()
    for frame in reader:
        count += 1
        sizes.add(tuple(frame.shape))
    reader.close()
    with Image.open(output / 'comparison.gif') as gif:
        gif_count, gif_duration_ms = gif.n_frames, 0
        for i in range(gif_count):
            gif.seek(i)
            gif.load()
            gif_duration_ms += gif.info['duration']
    if gif_count != len(gif_frames) or gif_duration_ms != 100 * len(gif_frames):
        raise RuntimeError('GIF preview decode/duration validation failed')
    validation = {'decoded_frames': count, 'shapes': sorted(sizes), 'fps': 10,
                  'duration_s': count / 10, 'sha256': sha(output / 'comparison.mp4'),
                  'max_scene_age_s': max(f['age_s'] for r in timeline for f in r['scene_frames'].values()),
                  'all_frame_hashes_verified': True, 'all_reference_arrays_identical': True,
                  'all_five_cameras_identical': True, 'camera_pose_tolerance': 1e-5,
                  'native_frame_count': native_frame_count,
                  'reference_centre_crossings_s': crossings,
                  'all_image_timestamps_equal_paused_truth': True,
                  'gif_preview': {'size_px': [960, 600], 'playback_speed': 2,
                                  'frame_count': len(gif_frames),
                                  'verified_duration_s': gif_duration_ms / 1000,
                                  'palette': 'fixed 64 adaptive plus reserved controller/marker colours',
                                  'sha256': sha(output / 'comparison.gif')},
                  'render_software': revision(), 'capture_software': manifest['software'],
                  'metrics': f'common {cfg["control_period"]:g} s grid from independent physics truth including {duration:g} s endpoint',
                  'alignment': 'At video time t, use latest acquired scene at or before t; plot truth at t',
                  'source': 'scripts/render_tracking_comparison.py', 'book_font': font}
    if count != len(video_times) or len(sizes) != 1:
        raise RuntimeError('Encoded video validation failed')
    (output / 'media_validation.json').write_text(dump(validation) + '\n')
    print(dump(validation))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    publish(a.source, a.output)
