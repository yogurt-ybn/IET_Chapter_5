"""Fit/train-only, validate on held-out flights, then freeze matched AirSim pairs."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from ..controllers.configuration import load_controller_configs
from ..controllers.nmpc.model import symbolic_model
from ..experiments.configuration import resolve
from ..experiments.engine import atomic_json, execute, source_fingerprint
from ..logging import revision
from .dataset import vehicle
from .gp import ForceGP, fit_force_gp, sha256


def read_dataset(directory):
    directory = Path(directory)
    manifest = json.loads((directory/'manifest.json').read_text())
    if len({f['flight_id'] for f in manifest['flights']}) != len(manifest['flights']):
        raise ValueError('Duplicate flight IDs in dataset')
    for flight in manifest['flights']:
        if sha256(directory/(flight['flight_id']+'.npz')) != flight['compact_sha256']:
            raise ValueError('Dataset changed after extraction')
    return manifest


def fit_models(dataset, output, support_count=64):
    dataset, output = Path(dataset), Path(output)
    manifest = read_dataset(dataset)
    output.mkdir(parents=True, exist_ok=False)
    models = {}
    for condition in sorted({f['condition'] for f in manifest['flights']}):
        train = [f for f in manifest['flights'] if f['condition'] == condition and f['split'] == 'train']
        test = [f for f in manifest['flights'] if f['condition'] == condition and f['split'] == 'validation']
        if not train or not test:
            raise ValueError('Each condition needs whole training and validation flights')
        samples = [dict(np.load(dataset/(f['flight_id']+'.npz'))) for f in train]
        masses = {f['controller_parameters']['mass'] for f in train+test}
        if len(masses) != 1:
            raise ValueError('One GP cannot mix different nominal controller masses')
        model = fit_force_gp(np.vstack([s['features'] for s in samples]),
                             np.vstack([s['target_force_N'] for s in samples]), support_count=support_count,
                             context={'nominal_mass_kg': masses.pop(), 'condition': condition,
                                 'dataset_manifest_sha256': sha256(dataset/'manifest.json'),
                                 'training_flights': [f['flight_id'] for f in train],
                                 'validation_flights': [f['flight_id'] for f in test],
                                 'source_sha256': source_fingerprint(), 'software': revision()})
        path = output/(condition+'.json')
        atomic_json(path, model.document)
        models[condition] = {'file': path.name, 'sha256': sha256(path)}
    atomic_json(output/'manifest.json', {'models': models, 'dataset_manifest_sha256': sha256(dataset/'manifest.json')})
    return models


def rk4_function(parameters, residual):
    import casadi as ca
    model = symbolic_model(parameters, 'validation', residual)
    rhs = ca.Function('rhs', [model.x, model.u], [model.f_expl_expr])
    x, u, h = ca.SX.sym('x', 17), ca.SX.sym('u', 4), ca.SX.sym('h')
    k1 = rhs(x, u)
    k2 = rhs(x+h/2*k1, u)
    k3 = rhs(x+h/2*k2, u)
    k4 = rhs(x+h*k3, u)
    return ca.Function('step', [x, u, h], [x+h/6*(k1+2*k2+2*k3+k4)])


def validate_models(dataset, models, output):
    dataset, models, output = Path(dataset), Path(models), Path(output)
    manifest = read_dataset(dataset)
    model_manifest = json.loads((models/'manifest.json').read_text())
    if model_manifest['dataset_manifest_sha256'] != sha256(dataset/'manifest.json'):
        raise ValueError('Models belong to another dataset')
    output.mkdir(parents=True, exist_ok=False)
    c = load_controller_configs()['nmpc']
    horizon = c['horizon']*c['design_dt']
    rows = []
    for flight in manifest['flights']:
        if flight['split'] != 'validation':
            continue
        entry = model_manifest['models'][flight['condition']]
        gp = ForceGP.load(models/entry['file'], entry['sha256'])
        if flight['flight_id'] in gp.document['context']['training_flights']:
            raise ValueError('Training/validation flight leakage')
        data = dict(np.load(dataset/(flight['flight_id']+'.npz')))
        X, Y = data['features'], data['target_force_N']
        prediction = gp.predict(X)
        dt = flight['physics_dt_s']
        steps = int(round(horizon/dt))
        one = int(round(c['design_dt']/dt))
        spacing = int(round(.24/dt))
        starts = np.arange(int(round(.12/dt)), len(data['t'])-steps, spacing)
        functions = [rk4_function(vehicle(flight['controller_parameters']), model) for model in (None, gp)]
        endpoint = np.empty((len(starts), 2, 2, 6))
        mse = np.zeros((2, steps+1, 2))
        for i, start in enumerate(starts):
            truth = data['x'][start:start+steps+1]
            for variant, function in enumerate(functions):
                x = truth[0].copy()
                for k in range(steps):
                    h = data['t'][start+k+1]-data['t'][start+k]
                    x = np.asarray(function(x, data['command'][start+k], h)).ravel()
                    if not np.isfinite(x).all():
                        raise ValueError('Prediction rollout became nonfinite; validation failed')
                    # Quaternion normalization is common to both offline predictors.
                    x[6:10] /= np.linalg.norm(x[6:10])
                    mse[variant, k+1] += [np.sum((x[:3]-truth[k+1, :3])**2), np.sum((x[3:6]-truth[k+1, 3:6])**2)]
                    if k+1 == one:
                        endpoint[i, variant, 0] = x[:6]-truth[k+1, :6]
                endpoint[i, variant, 1] = x[:6]-truth[-1, :6]
        if not len(starts):
            raise ValueError('No complete prediction horizons')
        curve = np.sqrt(mse/len(starts))
        np.savez_compressed(output/(flight['flight_id']+'.npz'), start_t=data['t'][starts],
                            endpoint_errors=endpoint, horizon_t=np.arange(steps+1)*dt,
                            error_curve=curve, label_t=data['label_t'], target_force_N=Y, predicted_force_N=prediction)
        row = {'flight_id': flight['flight_id'], 'condition': flight['condition'], 'trajectory': flight['trajectory'],
            'model_sha256': entry['sha256'], 'rollouts': len(starts),
            'one_step_s': one*dt, 'horizon_s': steps*dt,
            'force_label_baseline_rmse_N': np.sqrt(np.mean(Y**2, axis=0)).tolist(),
            'force_label_gp_rmse_N': np.sqrt(np.mean((Y-prediction)**2, axis=0)).tolist(),
            'one_step_position_rmse_m': curve[:, one, 0].tolist(),
            'one_step_velocity_rmse_m_s': curve[:, one, 1].tolist(),
            'horizon_position_rmse_m': curve[:, -1, 0].tolist(),
            'horizon_velocity_rmse_m_s': curve[:, -1, 1].tolist()}
        rows.append(row)
        print(f"Validated {flight['flight_id']}: position horizon nominal/GP {row['horizon_position_rmse_m']}", flush=True)
    report = {'schema':'iet-ch5/gp-validation-v1', 'dataset_manifest_sha256':sha256(dataset/'manifest.json'),
        'models_manifest_sha256':sha256(models/'manifest.json'), 'software':revision(), 'source_sha256':source_fingerprint(),
        'method':'free 17-state RK4 rollout from measured initial state; replay recorded future commanded wrenches, no truth-state corrections; applied motor lag predicted',
        'variant_order':['nominal','gp'], 'rollout_spacing_s':.24, 'rows':rows,
        'caveat':'open-loop recorded-input prediction validation, not new closed-loop flight; overlapping horizons are not independent statistical samples'}
    atomic_json(output/'REPORT.json', report)
    return report


def matched_pair_configuration(off, on):
    a, b = deepcopy(off), deepcopy(on)
    if a['nmpc']['residual']['enabled'] or not b['nmpc']['residual']['enabled']:
        raise ValueError('Expected residual disabled/enabled pair')
    a['nmpc']['residual']['enabled'] = True
    if a != b:
        raise ValueError('Paired controllers differ beyond the residual enable flag')
    return True


def compare(models, validation, output, local_config, suite_path, *, exclusive=False):
    if not exclusive:
        raise ValueError('An explicitly exclusive simulator is required')
    models, validation, output = Path(models), Path(validation), Path(output)
    model_manifest = json.loads((models/'manifest.json').read_text())
    checked = json.loads((validation/'REPORT.json').read_text())
    if checked['models_manifest_sha256'] != sha256(models/'manifest.json'):
        raise ValueError('Validation does not match frozen models')
    suite = json.loads(Path(suite_path).read_text())
    expected = {(c['id'], t, r) for c in suite['conditions'] for t in suite['trajectories'] for r in range(suite['repeats'])}
    validated_conditions = {r['condition'] for r in checked['rows']}
    if not {c['id'] for c in suite['conditions']} <= validated_conditions:
        raise ValueError('Every condition requires independent prediction validation first')
    output.mkdir(parents=True, exist_ok=False)
    receipt = {'software':revision(), 'source_sha256':source_fingerprint(), 'suite':suite,
               'models_manifest_sha256':sha256(models/'manifest.json'),
               'validation_report_sha256':sha256(validation/'REPORT.json'),
               'pairs':[], 'variant_order':'off/on in repeat 0, on/off in repeat 1; repeated for each condition/trajectory',
               'sole_controller_difference':'nmpc.residual.enabled; same model file and hash supplied to both variants'}
    atomic_json(output/'protocol.json',receipt)
    for condition in suite['conditions']:
        entry = model_manifest['models'][condition['id']]
        # Force model identity is checked even when preparing the disabled baseline.
        ForceGP.load(models/entry['file'], entry['sha256'])
        for trajectory in suite['trajectories']:
            for repeat in range(suite['repeats']):
                variants = {}
                for enabled in (False, True):
                    c = load_controller_configs()
                    c['nmpc']['residual'] = {'enabled':enabled,'model_path':str(models/entry['file']), 'model_sha256':entry['sha256']}
                    variants[enabled] = c
                matched_pair_configuration(variants[False], variants[True])
                pair = {'condition':condition['id'],'trajectory':trajectory,'repeat':repeat,'runs':{}}
                for enabled in ((False,True) if repeat%2 == 0 else (True,False)):
                    variant = 'on' if enabled else 'off'
                    resolved = resolve(suite_path, controllers=['nmpc'],trajectories=[trajectory],repeats=1)
                    resolved['suite']['conditions'] = [condition]
                    resolved['controllers'] = variants[enabled]
                    resolved['matrix'] = [item for item in resolved['matrix'] if item['condition'] == condition['id']]
                    if len(resolved['matrix']) != 1:
                        raise ValueError('Expected one frozen paired flight')
                    item = resolved['matrix'][0]
                    item['repeat'] = repeat
                    item['run_id'] = f'nmpc-{trajectory}-{condition["id"]}-r{repeat}-{variant}'
                    group = f'{condition["id"]}-{trajectory}-r{repeat}-{variant}'
                    journal = execute(resolved,local_config,output/group)
                    pair['runs'][variant] = {'group':group,'run_id':item['run_id'],'evaluation':journal[item['run_id']]}
                    atomic_json(output/'pending_pair.json',pair)
                receipt['pairs'].append(pair)
                atomic_json(output/'protocol.json',receipt)
    if {(p['condition'],p['trajectory'],p['repeat']) for p in receipt['pairs']} != expected:
        raise ValueError('Incomplete matched matrix')
    (output/'pending_pair.json').unlink(missing_ok=True)
    return receipt
