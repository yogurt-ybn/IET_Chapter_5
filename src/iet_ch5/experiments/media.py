"""Synchronized genuine simulator footage; stopped runs stay visibly stopped."""

import hashlib
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .engine import atomic_json
from .publication import COLORS


def render(source, output, rows, data):
    from ..references.tasks import Task
    if len({r['trajectory'] for r in rows}) != 1 or len({r['controller'] for r in rows}) != len(rows) or len(rows) > 6:
        raise ValueError('Media requires one trajectory, one run per controller, at most six controllers')
    frame_sets, pose = {}, None
    config = None
    total_native_frames = 0
    for row in rows:
        key = row['run_id']
        run_dir = source/'runs'/key
        meta = json.loads((run_dir/'metadata.json').read_text())
        current = meta['config']['task_config']
        if config is not None and current != config:
            raise ValueError('Demo references/conditions differ')
        config = current
        frames = json.loads((run_dir/'frames.json').read_text())
        times = np.array([f['simulation_t'] for f in frames])
        if not len(times) or not np.all(np.diff(times) > 0):
            raise ValueError('Missing/nonmonotonic actual scene frames')
        for f in frames:
            if hashlib.sha256((run_dir/'frames'/f['filename']).read_bytes()).hexdigest() != f['sha256']:
                raise ValueError('Scene checksum mismatch')
            if f['image_timestamp_ns'] != f['truth_timestamp_ns']:
                raise ValueError('Image/truth timestamp mismatch')
            p = f['camera_info']['pose']
            values = [p['position'][k+'_val'] for k in 'xyz']+[p['orientation'][k+'_val'] for k in 'xyzw']
            if pose is None:
                pose = values
            if not np.allclose(values, pose, atol=1e-5, rtol=0):
                raise ValueError('Fixed camera differs across demo runs')
        frame_sets[key] = frames, times, run_dir
        total_native_frames += len(frames)
    task = Task(rows[0]['trajectory'], config)
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    try:
        font, small = ImageFont.truetype(font_path, 20), ImageFont.truetype(font_path, 15)
    except OSError:
        font = small = ImageFont.load_default()
    timeline, previews = [], []
    output = Path(output)
    writer = imageio.get_writer(output/'comparison.mp4', fps=10, codec='libx264', quality=7,
                                macro_block_size=1, ffmpeg_log_level='error')
    poster_saved = False
    try:
        for index, t in enumerate(np.arange(0, task.duration+.001, .1)):
            canvas = Image.new('RGB', (1440, 920), '#101923')
            draw = ImageDraw.Draw(canvas)
            draw.text((20, 12), f"{rows[0]['trajectory']} | actual AirSim | t = {t:.1f} s | playback 1x", font=font, fill='white')
            draw.text((20, 44), 'Yellow: reference  |  Cyan: flown trace  |  Same fixed camera  |  Speed from physics truth', font=small, fill='#cbd5e1')
            record = {'video_t': float(t), 'frames': {}}
            for i, row in enumerate(rows):
                key, method = row['run_id'], row['controller']
                frames, times, run_dir = frame_sets[key]
                j = max(0, int(np.searchsorted(times, t+1e-9, side='right')-1))
                f = frames[j]
                x, y = (i%3)*480, 85+(i//3)*395
                image = Image.open(run_dir/'frames'/f['filename']).convert('RGB').resize((472, 354))
                canvas.paste(image, (x+4, y+36))
                error = np.linalg.norm(np.array(f['truth_p_nwu_m'])-f['reference_p_nwu_m'])
                speed = np.linalg.norm(f['truth_v_nwu_m_s'])
                stopped = not row['metrics'].get('complete_common_interval') and t > times[-1]+.12
                label = f'{method.upper()} | v={speed:.2f} m/s | e={error:.3f} m'
                draw.text((x+8, y+5), label, font=small, fill=COLORS[method])
                if len(rows) == 6:
                    draw.rectangle((x+5, y+369, x+470, y+390), fill='#101923')
                    draw.text((x+10, y+371), row['status'], font=small, fill='white')
                if stopped:
                    draw.rectangle((x+15, y+65, x+455, y+120), fill='#991b1b')
                    draw.text((x+24, y+80), 'STOPPED - LAST REAL FRAME HELD', font=small, fill='white')
                record['frames'][method] = {'filename': f['filename'], 'acquisition_t': f['simulation_t'], 'held_after_abort': stopped}
            if len(rows) < 6:
                # Sixth tile: reference geometry and measured outcome, not simulated footage.
                draw.rectangle((966, 480, 1434, 865), fill='#182536')
                draw.text((984, 496), 'TOP-DOWN REFERENCE / RESULTS', font=font, fill='white')
                refs = np.array([task.at(float(q)).p for q in np.linspace(0, task.duration, 401)])
                xy = np.column_stack([refs[:, 0], -refs[:, 1]])
                xy -= xy.min(axis=0)
                xy *= min(400/max(xy[:, 0].max(), .1), 130/max(xy[:, 1].max(), .1))
                points = [(float(a+990), float(b+542)) for a, b in xy]
                draw.line(points, fill='yellow', width=2)
                draw.text((984, 694), 'Analytic reference: NWU x/y (metres)', font=small, fill='#cbd5e1')
                for i, row in enumerate(rows):
                    rms = row['metrics'].get('tracking_rms_m')
                    label = f"{row['controller']}: {row['status']} | RMS {rms:.3f} m" if rms is not None else f"{row['controller']}: {row['status']}"
                    draw.text((984, 723+25*i), label, font=small, fill=COLORS[row['controller']])
            draw.text((20, 886), 'Separate capture runs; paused simulation timing. Failures retained. No physical-flight or real-time claim.', font=small, fill='#cbd5e1')
            writer.append_data(np.array(canvas))
            if index%2 == 0:
                previews.append(canvas.resize((864, 552)))
            if not poster_saved and t >= task.duration*.55:
                canvas.save(output/'comparison_preview.png')
                poster_saved = True
            timeline.append(record)
    finally:
        writer.close()
    palette = previews[len(previews)//2].quantize(colors=96)
    indexed = [p.quantize(palette=palette, dither=Image.Dither.NONE) for p in previews]
    indexed[0].save(output/'comparison.gif', save_all=True, append_images=indexed[1:], duration=100, loop=0, optimize=True)
    decoded = 0
    with imageio.get_reader(output/'comparison.mp4') as reader:
        for frame in reader:
            decoded += 1
    if decoded != len(timeline):
        raise ValueError('Encoded video did not decode completely')
    atomic_json(output/'video_timeline.json', timeline)
    atomic_json(output/'media_validation.json', {'native_frames_checked': total_native_frames,
        'fixed_camera_pose': pose, 'decoded_mp4_frames': decoded, 'mp4_simulation_speed': 1,
        'gif_simulation_speed': 2, 'frame_hashes_and_truth_timestamps_verified': True,
        'failed_runs': [r['run_id'] for r in rows if r['status'] != 'passed']})
