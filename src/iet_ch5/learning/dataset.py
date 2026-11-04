"""Flight-level data split and explicitly aligned applied-thrust discrepancy labels."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..allocation import Allocator
from ..frames import state_from_sample
from ..types import VehicleParameters
from ..logging import dump
from .gp import sha256

ARRAY_FIELDS = ('J', 'rotor_positions', 'rotor_normals', 'rotor_directions')


def vehicle(document):
    return VehicleParameters(**{k: np.asarray(v) if k in ARRAY_FIELDS else v for k, v in document.items()})


def aligned_flight(path, label_period=.024, warmup=.12):
    path = Path(path)
    meta = json.loads((path/'metadata.json').read_text())
    rows = [json.loads(line) for line in (path/'log.jsonl').read_text().splitlines()]
    physical = [json.loads(line) for line in (path/'physics.jsonl').read_text().splitlines()]
    start = meta['verified_initial_state']
    stamp = start['timestamp_ns']
    physical = [r for r in physical if r['timestamp'] > stamp]
    if not rows or not physical or any(r['has_collided'] for r in physical):
        raise ValueError('Missing/collided collection flight; retained raw attempt cannot train')
    states = [state_from_sample(r) for r in physical]
    times_ns = np.r_[stamp, np.array([s.timestamp_ns for s in states], dtype=np.int64)]
    if (np.any(np.diff(times_ns) <= 0) or np.max(np.diff(times_ns))-np.min(np.diff(times_ns)) > 1000
            or np.any(np.diff([r['sequence'] for r in physical]) != 1)):
        raise ValueError('Nonmonotonic/skipped physics samples')
    times = (times_ns-stamp)*1e-9
    R = np.array([start['R']]+[s.R for s in states])
    p = np.array([start['p']]+[s.p for s in states])
    v = np.array([start['v']]+[s.v for s in states])
    omega = np.array([start['omega']]+[s.omega for s in states])
    allocator = Allocator(vehicle(meta['parameters']))
    applied = np.vstack([rows[0]['telemetry_estimated_wrench']]+[allocator.from_telemetry(r['rotors']) for r in physical])
    ct = np.array([r['simulation_timestamp_ns'] for r in rows], dtype=np.int64)
    command_index = np.maximum(0, np.searchsorted(ct, times_ns, side='right')-1)
    commands = np.array([r['requested_wrench'] for r in rows])[command_index]
    mass, gravity = meta['controller_parameters']['mass'], meta['controller_parameters']['gravity']
    nominal_accel = applied[:, :1]/mass*R[:, :, 2]-[0, 0, gravity]
    vb = np.einsum('nji,nj->ni', R, v)
    dt = float(np.median(np.diff(times)))
    stride = int(round(label_period/dt))
    if stride < 1 or abs(stride*dt-label_period) > 1e-5:
        raise ValueError('Label period must match physics grid')
    X, Y, label_times = [], [], []
    for i in range(0, len(times)-stride, stride):
        if times[i] < warmup:
            continue
        j = i+stride
        interval = times[j]-times[i]
        mid = Rotation.from_rotvec(.5*Rotation.from_matrix(R[i].T@R[j]).as_rotvec()).as_matrix()
        Rmid = R[i]@mid
        nominal = np.trapezoid(nominal_accel[i:j+1], times[i:j+1], axis=0)/interval
        discrepancy = mass*Rmid.T@((v[j]-v[i])/interval-nominal)
        feature = np.r_[np.trapezoid(vb[i:j+1], times[i:j+1], axis=0)/interval,
                        np.trapezoid(applied[i:j+1, 0], times[i:j+1])/interval]
        X.append(feature)
        Y.append(discrepancy)
        label_times.append((times[i]+times[j])/2)
    xyzw = Rotation.from_matrix(R).as_quat()
    q = np.column_stack([xyzw[:, 3], xyzw[:, :3]])
    x = np.column_stack([p, v, q, omega, applied])
    arrays = {'t': times, 'timestamp_ns': times_ns, 'x': x, 'command': commands,
              'features': np.array(X), 'target_force_N': np.array(Y), 'label_t': np.array(label_times)}
    receipt = {'flight_id': path.name, 'condition': meta['config']['task_config']['condition']['id'],
        'trajectory': meta['config']['task'], 'software': meta['software'], 'config_sha256': meta['config_sha256'],
        'physics_samples': len(times), 'physics_dt_s': dt, 'label_period_s': stride*dt, 'warmup_excluded_s': warmup,
        'parameters': meta['parameters'], 'controller_parameters': meta['controller_parameters'],
        'raw_files': {n: {'bytes': (path/n).stat().st_size, 'sha256': sha256(path/n)} for n in ('log.jsonl','physics.jsonl','metadata.json','result.json')},
        'label_definition': 'm_nom R_mid.T [(v_end-v_start)/dt - trapezoid(f_applied R e3/m_nom-g e3)/dt]',
        'input_definition': 'interval mean body velocity and telemetry-reconstructed applied collective',
        'caveat': 'label includes unmodeled loads, discretization and telemetry phase; not measured external force'}
    return arrays, receipt


def build_dataset(source, output, training_trajectories, validation_trajectories):
    source, output = Path(source), Path(output)
    if set(training_trajectories)&set(validation_trajectories):
        raise ValueError('Training and validation trajectories must be disjoint')
    progress = json.loads((source/'progress.json').read_text())
    output.mkdir(parents=True, exist_ok=False)
    flights = []
    for row in progress.values():
        if row['status'] not in ('passed', 'tracking_failed') or not row['metrics']['complete_common_interval']:
            raise ValueError('Incomplete collection attempt: retain it and collect a new complete flight')
        trajectory = row['trajectory']
        split = 'train' if trajectory in training_trajectories else 'validation' if trajectory in validation_trajectories else None
        if split is None:
            raise ValueError('Every collection flight requires a declared split')
        arrays, receipt = aligned_flight(source/'runs'/row['run_id'])
        np.savez_compressed(output/(row['run_id']+'.npz'), **arrays)
        receipt.update(split=split, compact_sha256=sha256(output/(row['run_id']+'.npz')))
        flights.append(receipt)
    document = {'schema':'iet-ch5/residual-dataset-v1', 'flights':flights,
                'split_policy':'whole flights and disjoint trajectory families; no row-level random split',
                'training_trajectories':training_trajectories,'validation_trajectories':validation_trajectories}
    (output/'manifest.json').write_text(dump(document)+'\n')
    return document
