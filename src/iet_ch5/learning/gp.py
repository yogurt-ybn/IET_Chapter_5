"""Original small subset-of-data GP: standard RBF posterior mean, no online fitting.

Inspired by the GP residual workflow of Torrente et al. (2021); no upstream
source is copied. Predictions are body-frame force discrepancy [N], not wind.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.linalg import cho_factor, cho_solve
from scipy.optimize import minimize


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class ForceGP:
    def __init__(self, document):
        if document['schema'] != 'iet-ch5/force-gp-v1':
            raise ValueError('Unsupported force GP schema')
        self.document = document
        self.center = np.asarray(document['input_center'], dtype=float)
        self.scale = np.asarray(document['input_scale'], dtype=float)
        self.support = np.asarray(document['support_normalized'], dtype=float)
        self.alpha = np.asarray(document['alpha'], dtype=float)
        self.length = np.asarray(document['length_scales'], dtype=float)
        self.amplitude = np.asarray(document['amplitude'], dtype=float)
        self.mean = np.asarray(document['output_mean_N'], dtype=float)
        self.output_scale = np.asarray(document['output_scale_N'], dtype=float)
        self.bound = float(document['force_bound_N'])
        n = len(self.support)
        if (self.center.shape != (4,) or self.scale.shape != (4,)
                or self.support.shape != (n, 4) or self.alpha.shape != (n, 3)
                or self.length.shape != (3, 4) or self.amplitude.shape != (3,)
                or self.mean.shape != (3,) or self.output_scale.shape != (3,)
                or n < 1 or not np.isfinite(self.bound) or self.bound <= 0):
            raise ValueError('Invalid force GP dimensions or bound')
        for value in (self.center, self.scale, self.support, self.alpha, self.length,
                      self.amplitude, self.mean, self.output_scale):
            if not np.isfinite(value).all():
                raise ValueError('Nonfinite GP artifact')
        if any(np.any(a <= 0) for a in (self.scale, self.length, self.amplitude, self.output_scale)):
            raise ValueError('Nonpositive GP scales')

    @classmethod
    def load(cls, path, expected_hash):
        if not expected_hash or sha256(path) != expected_hash:
            raise ValueError('Force GP model checksum mismatch')
        return cls(json.loads(Path(path).read_text()))

    def predict(self, features):
        features = np.atleast_2d(features)
        z = (features-self.center)/self.scale
        outputs = []
        for axis in range(3):
            distance = (z[:, None, :]-self.support[None, :, :])/self.length[axis]
            kernel = self.amplitude[axis]**2*np.exp(-.5*np.sum(distance**2, axis=2))
            outputs.append(self.mean[axis]+self.output_scale[axis]*(kernel@self.alpha[:, axis]))
        return np.clip(np.column_stack(outputs), -self.bound, self.bound)

    def symbolic(self, features):
        import casadi as ca
        z = (features-ca.DM(self.center))/ca.DM(self.scale)
        values = []
        for axis in range(3):
            kernel = []
            for support in self.support:
                d = (z-ca.DM(support))/ca.DM(self.length[axis])
                kernel.append(self.amplitude[axis]**2*ca.exp(-.5*ca.dot(d, d)))
            force = self.mean[axis]+self.output_scale[axis]*ca.dot(ca.vertcat(*kernel), ca.DM(self.alpha[:, axis]))
            values.append(ca.fmin(self.bound, ca.fmax(-self.bound, force)))
        return ca.vertcat(*values)


def fit_force_gp(features, targets, *, support_count=64, force_bound=3., context=None):
    """Exact independent-output GP on a deterministic train-only subset, not variational GP."""
    X, Y = np.asarray(features), np.asarray(targets)
    if (X.ndim != 2 or X.shape[1] != 4 or Y.shape != (len(X), 3) or len(X) < 8
            or not isinstance(support_count, int) or support_count < 1):
        raise ValueError('Need aligned training features (n,4) and force targets (n,3)')
    if not np.isfinite(X).all() or not np.isfinite(Y).all():
        raise ValueError('Nonfinite training samples')
    center, scale = X.mean(axis=0), np.maximum(X.std(axis=0), [.1, .1, .1, .1])
    Z = (X-center)/scale
    # Farthest-point subset includes the training mean's closest observation.
    indices = [int(np.argmin(np.sum(Z**2, axis=1)))]
    distance = np.sum((Z-Z[indices[0]])**2, axis=1)
    for _ in range(min(support_count, len(X))-1):
        index = int(np.argmax(distance))
        if distance[index] < 1e-14:
            break
        indices.append(index)
        distance = np.minimum(distance, np.sum((Z-Z[index])**2, axis=1))
    support = Z[indices]
    means, scales = Y.mean(axis=0), np.maximum(Y.std(axis=0), .005)
    normalized = (Y[indices]-means)/scales
    differences = support[:, None, :]-support[None, :, :]
    alpha, lengths, amplitudes, noises, optimizers = [], [], [], [], []
    for axis in range(3):
        y = normalized[:, axis]
        def matrices(theta):
            length, amplitude, noise = np.exp(theta[:4]), np.exp(theta[4]), np.exp(theta[5])
            K = amplitude**2*np.exp(-.5*np.sum((differences/length)**2, axis=2))
            return K+(noise**2+1e-8)*np.eye(len(support))
        def objective(theta):
            factor = cho_factor(matrices(theta), lower=True)
            return float(.5*y@cho_solve(factor, y)+np.log(np.diag(factor[0])).sum()+.5*len(y)*np.log(2*np.pi))
        result = minimize(objective, np.log([2., 2., 2., 2., 1., .05]), method='L-BFGS-B',
                          bounds=[(np.log(.2), np.log(30.))]*4+[(np.log(.02), np.log(10.)), (np.log(.001), np.log(1.))],
                          options={'maxiter': 150})
        theta = result.x
        alpha.append(cho_solve(cho_factor(matrices(theta), lower=True), y))
        lengths.append(np.exp(theta[:4]))
        amplitudes.append(np.exp(theta[4]))
        noises.append(np.exp(theta[5]))
        optimizers.append({'success': bool(result.success), 'message': str(result.message), 'nll': float(result.fun)})
    return ForceGP({'schema': 'iet-ch5/force-gp-v1', 'features': ['body_vx_m_s', 'body_vy_m_s', 'body_vz_m_s', 'applied_collective_N'],
        'target': 'body force model discrepancy N, reconstructed from physics motion and applied rotor telemetry; not measured wind',
        'input_center': center.tolist(), 'input_scale': scale.tolist(), 'support_normalized': support.tolist(),
        'support_indices': indices, 'alpha': np.column_stack(alpha).tolist(), 'length_scales': np.array(lengths).tolist(),
        'amplitude': amplitudes, 'noise_normalized': noises, 'output_mean_N': means.tolist(), 'output_scale_N': scales.tolist(),
        'force_bound_N': force_bound, 'optimizer': optimizers, 'context': context or {},
        'uncertainty_policy': 'posterior mean only; no chance constraints or calibrated uncertainty claim'})
