"""Common simulation-time delay and saved held-Gaussian feedback noise realization."""

from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

from .types import State


class FeedbackPipeline:
    def __init__(self, config, duration, seed, initial_state):
        self.c = config
        self.history = deque([initial_state])
        self.t0 = initial_state.timestamp_ns
        self.noise_period = config["noise_period"]
        rng = np.random.default_rng(seed)
        self.noise = rng.standard_normal((int(duration/self.noise_period)+3, 12))
        self.scales = np.repeat([config["position_std_m"], config["velocity_std_m_s"],
                                  config["attitude_std_rad"], config["rate_std_rad_s"]], 3)
        self.noise *= self.scales

    def add_truth(self, states):
        self.history.extend(states)

    def sample(self, truth):
        desired_time = truth.timestamp_ns - int(self.c["delay_s"]*1e9)
        while len(self.history) >= 2 and self.history[1].timestamp_ns <= desired_time:
            self.history.popleft()
        left = self.history[0]
        right = self.history[1] if len(self.history) > 1 else left
        alpha = np.clip((desired_time-left.timestamp_ns)/max(1,right.timestamp_ns-left.timestamp_ns),0,1)
        p, v, omega = [(1-alpha)*getattr(left,k)+alpha*getattr(right,k) for k in ["p","v","omega"]]
        delta = Rotation.from_matrix(left.R.T @ right.R).as_rotvec()
        R = left.R @ Rotation.from_rotvec(alpha*delta).as_matrix()
        index = min(int((truth.timestamp_ns-self.t0)*1e-9/self.noise_period), len(self.noise)-1)
        n = self.noise[index]
        return State(p+n[:3], v+n[3:6], R @ Rotation.from_rotvec(n[6:9]).as_matrix(),
                     omega+n[9:12], max(self.t0,desired_time),
                     "Shared truth interpolation, configured delay and saved Gaussian noise"), index
