"""Paired augmented random search on a frozen identified simulator, seed 731."""

import argparse
from pathlib import Path
import numpy as np
from iet_ch5.learning.dataset import vehicle
from iet_ch5.allocation import Allocator
from iet_ch5.trajectories import Trajectory, load_trajectory
from iet_ch5.controllers.lqr.controller import reference_vector
from controllers import policy_matrix, START
import json


def train(model, metadata, destination):
    params = vehicle(json.loads(metadata.read_text())["parameters"])
    allocator = Allocator(params)
    data = np.load(model)
    A, B, bias, hover = [data[k] for k in ("A", "B", "bias", "hover")]
    references = {}
    for name in ("circle", "helix", "transfer"):
        task = Trajectory(load_trajectory(name))
        ref = [task.at(t) for t in np.arange(0, task.duration, 0.012)]
        references[name] = (
            np.array([reference_vector(r, params) for r in ref]),
            np.array([params.mass * r.a[2] for r in ref]),
        )
    rng = np.random.default_rng(731)
    theta = np.zeros(len(START))
    history = []
    for epoch in range(90):
        name = list(references)[epoch % 3]
        reference, acceleration = references[name]
        start = rng.integers(0, len(reference) - 300)
        initial = np.r_[reference[start], np.zeros(4)]
        initial[:6] += rng.normal(0, [0.12, 0.12, 0.12, 0.15, 0.15, 0.15])
        noise = rng.normal(size=(24, len(theta)))
        candidates = np.vstack((theta + 0.07 * noise, theta - 0.07 * noise, theta))
        weights = policy_matrix(candidates)
        state = np.tile(initial, (len(weights), 1))
        returns = np.zeros(len(weights))
        for i in range(start, start + 300):
            error = state[:, :12] - reference[i]
            action = np.einsum("nij,nj->ni", weights, error)
            action[:, 0] += acceleration[i]
            rotor = np.clip((action + hover) @ allocator.inverse.T, 0, 1)
            applied = rotor @ allocator.B.T - hover
            returns -= (
                np.sum(error[:, :3] ** 2, axis=1)
                + 0.04 * np.sum(error[:, 3:6] ** 2, axis=1)
                + 0.0002 * np.sum((action / [1, 0.05, 0.05, 0.02]) ** 2, axis=1)
            )
            state = state @ A.T + applied @ B.T + bias
        plus, minus = returns[:24], returns[24:48]
        ids = np.argsort(np.maximum(plus, minus))[-12:]
        spread = np.std(np.r_[plus[ids], minus[ids]])
        theta += 0.045 * ((plus[ids] - minus[ids]) @ noise[ids]) / (12 * max(spread, 1e-8))
        theta = np.clip(theta, -2.5, 2.5)
        history.append(float(returns[-1]))
    np.savez(destination, theta=theta, training_return=history, seed=731)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for key in ["model", "metadata", "destination"]:
        p.add_argument(key, type=Path)
    args = p.parse_args()
    train(args.model, args.metadata, args.destination)
