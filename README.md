# IET Chapter 5: UAV Control Systems

AirSim implementations and reproducible case-study materials for the conventional UAV control methods in Chapter 5.

## Status

M0 bridge validation and M1 genuine AirSim hover/transfer acceptance passed for PID, LQR
and standard SE(3). See [the environment report](docs/environment_report.md) and
[M1 acceptance, evidence and limitations](docs/m1_acceptance.md). Observer/MPC and the
fixed comparison matrix are pending. No controller ranking or real-time feasibility is claimed.

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

## Install and verify

Python 3.10 is the tested interpreter. From this repository:

```bash
uv sync --extra dev --extra plots --extra mpc --locked
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest -q
.venv/bin/ruff check src scripts tests
```

Without an explicitly selected server, integration tests skip. The host's inherited ROS
pytest plugins are disabled; no host package upgrades are needed.

## Select and launch an isolated simulator

Reuse an existing Unreal 4.27.2 / AirSim Blocks installation. Inspect running experiments
first. Create **ignored** `.local/environment.json` with keys `editor` (absolute installed
UE4Editor executable), `project` (absolute Blocks.uproject), `airsim_root`, `host` (local
RPC bind address), `port` (a free dedicated port matching configs/airsim-settings.json),
and `vehicle` (`Chapter5`). Do not commit this local file. The telemetry extension patch
and observed build are documented in the environment report; upstream AirSim alone does
not guarantee these custom APIs. No rebuild is needed on the already validated host.

```bash
.venv/bin/python scripts/launch_simulator.py --local-config .local/environment.json
.venv/bin/python scripts/check_environment.py --local-config .local/environment.json --output .local/capabilities.json
```

The launcher refuses an occupied RPC port and writes its own PID/settings/logs to `.local`.
After the server finishes loading, the read-only check must report the required dynamics
APIs available. It never arms, resets, pauses or commands the vehicle.

The following command **resets, arms and operates only the selected simulator**. Use an
exclusive instance. It finishes paused and disarmed:

```bash
.venv/bin/python scripts/bridge_smoke.py --local-config .local/environment.json --exclusive-simulator --check-overflow --output .local/bridge-smoke.json
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 IET_CH5_LOCAL_CONFIG=.local/environment.json .venv/bin/pytest -q
```

The overflow check takes about 30 s of simulation time.

## Run a baseline and regenerate figures

```bash
.venv/bin/python scripts/run_experiment.py --local-config .local/environment.json --exclusive-simulator --controller pid --output results/runs/my-pid-run
.venv/bin/python scripts/plot_results.py results/runs/my-pid-run --output results/runs/my-pid-run/figures
.venv/bin/python scripts/plot_results.py results/examples/m1/pid results/examples/m1/lqr results/examples/m1/geometric --output results/examples/m1/figures
```

Use `--controller lqr` or `--controller geometric` for the other accepted baselines.
Output directories must be new, so an experiment cannot silently overwrite earlier data.
Every run resets controller memory and records its exact configuration, source revision,
truth/feedback, requested/estimated wrench, motor inputs/states, timing and failures.
The `circle` task is implemented but has not yet passed simulator acceptance at M1.
Frozen full-comparison commands will follow in M3.
