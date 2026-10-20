"""Record all five controllers tracking one predefined path through the motor bridge."""

import argparse
import json
from pathlib import Path

from iet_ch5.logging import config_hash, dump, revision
from iet_ch5.runner import run
from iet_ch5.scene_capture import SceneCapture


METHODS = ['pid', 'lqr', 'geometric', 'dob', 'mpc']


def capture(local_config, output, task_path):
    local = json.loads(local_config.read_text())
    controllers = json.loads(Path('configs/controllers.json').read_text())
    tasks = json.loads(task_path.read_text())
    task_name = tasks.get('demo_task', 'circle')
    if task_name not in ('circle', 'figure_eight'):
        raise ValueError('A comparison requires a circle or figure_eight task')
    output.mkdir(parents=True, exist_ok=False)
    manifest = {'schema': 'iet-ch5/tracking-demo-v1', 'software': revision(),
                'task_config': tasks, 'controllers_sha256': config_hash(controllers),
                'controllers': METHODS, 'task': task_name, 'capture_period_s': .1,
                'purpose': 'separate visual comparison, excluded from frozen benchmark timing',
                'reference': 'C4 smooth progress, fixed altitude/heading; all controllers use same task',
                'initialization': 'identical reset/rotor spinup/direct truth verification per run'}
    (output / 'manifest.json').write_text(dump(manifest) + '\n')
    summary = []
    for method in METHODS:
        print(f'Capturing {method} on the common reference', flush=True)
        result = run(local, controllers, tasks, method, task_name, output / method,
                     scene_capture=SceneCapture(
                         position=tasks.get('demo_camera_position_vehicle_origin_ned_m')))
        summary.append(result)
        (output / 'results.json').write_text(dump(summary) + '\n')
        print(dump(result), flush=True)
        if result.failure and result.failure != 'Tracking acceptance tolerance not met':
            raise RuntimeError(f'{method}: {result.failure}; evidence retained')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--local-config', type=Path, required=True)
    p.add_argument('--exclusive-simulator', action='store_true', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--task-config', type=Path, default=Path('configs/figure-eight-cruise-task.json'))
    a = p.parse_args()
    capture(a.local_config, a.output, a.task_config)
