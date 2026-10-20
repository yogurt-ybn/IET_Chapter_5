# IET Chapter 5: UAV Control Systems

AirSim implementations and reproducible case-study materials for the conventional UAV control methods in Chapter 5.

## Status

Requirements and initial upstream screening are available. Controller implementation and AirSim benchmark results are pending. No measured comparison is claimed yet.

## Start here

- [Implementation requirements and acceptance criteria](docs/IMPLEMENTATION_REQUIREMENTS.md)
- [Upstream code candidates and adaptation notes](docs/UPSTREAM_CANDIDATES.md)
- [Agent handoff](AGENTS.md)

## Planned controllers

1. Cascaded PID.
2. Near-hover linear state feedback/LQR.
3. Geometric attitude/trajectory control on SO(3)/SE(3).
4. Disturbance-observer compensation with a matched nominal-controller ablation.
5. Constrained model predictive control.

All methods will use a common state/reference contract, verified motor-level AirSim adapter, experiment configuration, logging and evaluation. Deliverables include reproducible code, source/licence notices, measured plots and actual simulator visualisations. Simulation-to-real deployment is a later extension.