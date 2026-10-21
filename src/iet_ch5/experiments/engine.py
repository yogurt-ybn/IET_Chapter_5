"""Frozen, resumable experiments; failed complete runs are never silently rerun."""

import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import time
import traceback

import numpy as np

from ..evaluation.reanalyze import reanalyze
from ..frames import C
from ..logging import dump, revision, config_hash
from ..runner import run
from ..scene_capture import SceneCapture
from ..simulation import create_backend
from ..trajectories.analysis import screen


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(dump(value)+'\n')
    temporary.replace(path)


def source_fingerprint():
    digest = hashlib.sha256()
    for root in ('src', 'scripts', 'configs'):
        for path in sorted(Path(root).rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix in ('.py', '.json'):
                digest.update(str(path).encode())
                digest.update(path.read_bytes())
    for filename in ('pyproject.toml', 'uv.lock'):
        path = Path(filename)
        if path.exists():
            digest.update(path.read_bytes())
    return digest.hexdigest()


def classify(result, metrics):
    if not metrics.get('complete_common_interval'):
        return 'aborted'
    return 'passed' if result.status == 'passed' else 'tracking_failed'


def execute(resolved, local_config, output, *, resume=False, demo=False):
    output = Path(output)
    if demo and (len(resolved['trajectories']) != 1 or len(resolved['matrix']) > 6
                 or len({r['controller'] for r in resolved['matrix']}) != len(resolved['matrix'])):
        raise ValueError('Demo requires one trajectory and one run per controller, at most six')
    frozen = {'resolved': resolved, 'source_sha256': source_fingerprint(), 'software': revision(),
              'capture_demo': demo, 'thread_environment': {k: os.environ.get(k) for k in
                  ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS')}}
    if output.exists():
        if not resume:
            raise FileExistsError('Use a new output folder or --resume')
        previous = json.loads((output/'protocol.json').read_text())
        if any(previous[k] != frozen[k] for k in ('resolved', 'source_sha256', 'capture_demo')):
            raise ValueError('Resume requires identical frozen configuration and executable source')
    else:
        output.mkdir(parents=True)
        atomic_json(output/'protocol.json', frozen)
    # Private lock name hashes the endpoint, without publishing its address.
    endpoint = config_hash({k: local_config.get(k) for k in ('host', 'port', 'vehicle')})
    lockdir = Path('.local/locks')
    lockdir.mkdir(parents=True, exist_ok=True)
    with (lockdir/endpoint).open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        backend = create_backend(local_config)
        try:
            preflight = [screen(t, backend.parameters) for t in resolved['trajectories']]
        finally:
            backend.close()
        atomic_json(output/'preflight.json', preflight)
        if any(not r['physical_feedforward_feasible'] for r in preflight):
            raise ValueError('Reference feedforward screen failed; inspect preflight.json')
        journal = json.loads((output/'progress.json').read_text()) if (output/'progress.json').exists() else {}
        previous_signal = signal.getsignal(signal.SIGTERM)
        def interrupt(signum, frame):
            raise KeyboardInterrupt('SIGTERM; incomplete run preserved')
        signal.signal(signal.SIGTERM, interrupt)
        try:
            for item in resolved['matrix']:
                key = item['run_id']
                if key in journal and journal[key]['status'] in ('passed', 'tracking_failed', 'aborted'):
                    continue
                path = output/'runs'/key
                if path.exists():
                    attempts = output/'attempts'
                    attempts.mkdir(exist_ok=True)
                    archived = attempts/f'{key}-{time.time_ns()}'
                    path.rename(archived)
                    atomic_json(archived/'attempt.json', {'reason': 'Incomplete/infrastructure attempt preserved before explicit resume'})
                journal[key] = {**{k: v for k, v in item.items() if k != 'task_config'}, 'status': 'running'}
                atomic_json(output/'progress.json', journal)
                try:
                    capture = None
                    if demo:
                        trajectory = item['task_config']['trajectory']
                        capture = SceneCapture(period=.12, position=C@np.array(trajectory['camera_position_nwu_m']))
                    result = run(local_config, resolved['controllers'], item['task_config'], item['controller'],
                                 item['trajectory'], path, seed=item['seed'], scene_capture=capture)
                    metrics, _ = reanalyze(path, path/'metrics_samples.npz')
                    status = classify(result, metrics)
                    # Offline endpoint-inclusive acceptance is authoritative for new experiments.
                    if metrics.get('complete_common_interval') and not result.failure:
                        status = 'passed' if metrics['tracking_rms_m'] < item['task_config']['hold_acceptance_rms_m'] else 'tracking_failed'
                    journal[key].update(status=status, failure=result.failure, metrics={**result.metrics, **metrics})
                    atomic_json(path/'evaluation.json', journal[key])
                except KeyboardInterrupt:
                    journal[key]['status'] = 'interrupted'
                    atomic_json(output/'progress.json', journal)
                    raise
                except Exception as error:
                    path.mkdir(parents=True, exist_ok=True)
                    (path/'infrastructure_failure.txt').write_text(traceback.format_exc())
                    journal[key].update(status='infrastructure_failed', failure=f'{type(error).__name__}: {error}')
                    atomic_json(output/'progress.json', journal)
                    raise
                atomic_json(output/'progress.json', journal)
                print(f"{sum(v['status'] != 'running' for v in journal.values())}/{len(resolved['matrix'])} {key}: {journal[key]['status']}", flush=True)
        finally:
            signal.signal(signal.SIGTERM, previous_signal)
    return journal
