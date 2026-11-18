"""Three independent ARS fits using the nonlinear predictor and expanded flight envelope."""

import json, sys
from pathlib import Path
import numpy as np
from split_routes import TRAIN, ROOT
from iet_ch5.trajectories import load_trajectory, Trajectory
from iet_ch5.learning.dataset import vehicle
from iet_ch5.allocation import Allocator
from wide_control import reference_grid, observation, DT
from wide_model import step
from controllers import policy_matrix, START


def train(seed, model, episodes, allocator, destination):
    rng = np.random.default_rng(seed)
    theta = np.zeros(len(START))
    history = []
    for epoch in range(135):
        ref, feed, limit = episodes[epoch % len(episodes)]
        n = 300
        start = rng.integers(0, limit - n)
        noise = rng.normal(size=(20, len(theta)))
        matrices = policy_matrix(np.vstack((theta + 0.065 * noise, theta - 0.065 * noise, theta)))
        x = np.tile(np.r_[ref[start], feed[start] - model["hover"]], (len(matrices), 1))
        x[:, :6] += rng.normal(0, [0.11, 0.11, 0.11, 0.15, 0.15, 0.15])
        total = np.zeros(len(x))
        alive = np.ones(len(x), dtype=bool)
        for j in range(start, start + n):
            error = observation(x, ref[j])
            wrench = np.einsum("nij,nj->ni", matrices, error) + feed[j]
            pwm = wrench @ allocator.inverse.T
            # Infeasible requests end the training episode; no fictitious clipped mixer.
            valid = np.all((pwm >= 0) & (pwm <= 1), axis=1) & (
                np.linalg.norm(error[:, :3], axis=1) < 2
            )
            total[alive & ~valid] -= 1.5 * (start + n - j)
            alive &= valid
            if not any(alive):
                break
            deviation = wrench - feed[j]
            cost = (
                np.sum(error[:, :3] ** 2, axis=1)
                + 0.035 * np.sum(error[:, 3:6] ** 2, axis=1)
                + 0.00015 * np.sum((deviation / [1, 0.05, 0.05, 0.02]) ** 2, axis=1)
            )
            total[alive] -= cost[alive]
            x[alive] = step(x[alive], wrench[alive] - model["hover"], model)
        plus, minus = total[:20], total[20:40]
        top = np.argsort(np.maximum(plus, minus))[-10:]
        std = np.std(np.r_[plus[top], minus[top]])
        theta += 0.035 * ((plus[top] - minus[top]) @ noise[top]) / (10 * max(1e-7, std))
        theta = theta.clip(-2.5, 2.5)
        history.append(total[-1])
    np.savez(destination, theta=theta, seed=seed, returns=history)


if __name__ == "__main__":
    out = Path(sys.argv[1])
    p = vehicle(json.loads((out / "vehicle.json").read_text()))
    with np.load(out / "nonlinear.npz") as data:
        model = dict(data)
    episodes = []
    for path in ["circle", "helix", "transfer", *[ROOT / (name + ".json") for name in TRAIN]]:
        traj = Trajectory(load_trajectory(path))
        episodes.append((*reference_grid(traj.at, traj.duration, p), int(traj.duration / DT)))
    np.savez(out / "initial.npz", theta=np.zeros(len(START)))
    for seed in (731, 732, 733):
        train(seed, model, episodes, Allocator(p), out / f"ars-{seed}.npz")
        print("trained", seed, flush=True)
