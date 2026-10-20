# IET Chapter 5: UAV Control Systems

AirSim implementations and reproducible case-study materials for the conventional UAV control methods in Chapter 5.

## Status

All five controllers are implemented through one verified motor interface. The fixed AirSim
case study contains **51 runs: 47 tracking passes and four retained model-mismatch failures**.
See the [English case study](docs/case_study.md), [all-run results](results/examples/m3/summary.csv)
and [figures](results/examples/m3/figures/matrix/accuracy_matrix.png).
The measured comparison uses paused simulation time. Real-time operation, physical flight
and a prescribed external force/moment pulse are not claimed.

## Start here

- [Implementation requirements and acceptance criteria](docs/IMPLEMENTATION_REQUIREMENTS.md)
- [Upstream code candidates and adaptation notes](docs/UPSTREAM_CANDIDATES.md)
- [Agent handoff](AGENTS.md)

## Implemented controllers

1. Cascaded PID.
2. Near-hover linear state feedback/LQR.
3. Geometric attitude/trajectory control on SO(3)/SE(3).
4. Disturbance-observer compensation with a matched nominal-controller ablation.
5. Constrained model predictive control.

All methods use the common state/reference contract, physical allocation, experiment runner,
logging and evaluation. See [conventions](docs/model_and_conventions.md),
[equation mapping](docs/controller_mapping.md), [source notices](THIRD_PARTY_NOTICES.md)
and [protocol](docs/experiment_protocol.md). No built-in AirSim position/velocity controller
is relabelled as a chapter algorithm.

## Install and verify

Python 3.10 is the tested interpreter. From this repository:

```bash
uv sync --extra dev --extra plots --extra mpc --locked
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
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

The overflow check advances until more than 10,000 new physics samples exist, about 30 s
of simulation time; wall duration can be longer. Read the [bridge regression diagnosis](docs/bridge_regression.md)
before replacing measured timestamps with a single wall-timed continuation call.

## Run a baseline and regenerate figures

```bash
.venv/bin/python scripts/run_experiment.py --local-config .local/environment.json --exclusive-simulator --controller pid --output results/runs/my-pid-run
.venv/bin/python scripts/plot_results.py results/runs/my-pid-run --output results/runs/my-pid-run/figures
.venv/bin/python scripts/plot_results.py results/examples/m1/pid results/examples/m1/lqr results/examples/m1/geometric --output results/examples/m1/figures
```

Controller choices are `pid`, `lqr`, `geometric`, `dob` and `mpc`. Tasks are
`hover_transfer`, `circle`, `disturbance`, `mismatch` and `noise_delay`.
Use `--dob-disabled` for the matched compensation-off ablation.
Output directories must be new, so an experiment cannot silently overwrite earlier data.
Every run resets controller memory and records its exact configuration, source revision,
truth/feedback, requested/estimated wrench, motor inputs/states, timing and failures.

## Full matrix and publication figures

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/run_benchmark.py --local-config .local/environment.json --exclusive-simulator --output results/runs/my-benchmark
.venv/bin/python scripts/summarize_benchmark.py --source results/runs/my-benchmark --output results/runs/my-analysis
.venv/bin/python scripts/plot_benchmark_summary.py --source results/runs/my-analysis --output results/runs/my-analysis/figures
```

The matrix takes several hundred seconds of physics time. It retains failures, freezes
source/configuration before running, and supports `--resume` only when the manifest matches.
The published numerical source is `2e6d9ed06c3083c17b4a3b074a1537e92bdb0880`; use that
commit in a separate checkout when reproducing the exact recorded version. Later commits
add common-duration analysis, artefacts and documentation. Two abandoned bridge-validation
attempts are kept separately and excluded from the final matrix.

Regenerate all-run figures entirely from committed compact data:

```bash
.venv/bin/python scripts/plot_benchmark_summary.py --source results/examples/m3 --output results/examples/m3/figures/matrix
.venv/bin/python scripts/plot_results.py results/examples/m3/runs/00-pid-hover_transfer-s0-r0 results/examples/m3/runs/10-lqr-hover_transfer-s0-r0 results/examples/m3/runs/20-geometric-hover_transfer-s0-r0 results/examples/m3/runs/30-dob-hover_transfer-s0-r0 results/examples/m3/runs/40-mpc-hover_transfer-s0-r0 --output results/examples/m3/figures/nominal
.venv/bin/python scripts/plot_results.py results/examples/m3/runs/33-dob-disturbance-s0-r0 results/examples/m3/runs/50-dob-disturbance-s0-r0-off --output results/examples/m3/figures/observer_ablation
```

Outputs include editable Python source plus PDF/SVG/PNG. Position statistics in the case
study use independent physics truth on identical complete simulation-time windows;
original controller-run statistics are also retained. Full control/physics/noise logs are
in the ignored local archive identified by [its checksum](results/examples/m3/full_archive.json).
Compact examples, all-run metric samples/configurations/results and regenerated figures
are committed; an external upload of the larger archive is not claimed.

## Actual AirSim demo

[Scene image](results/examples/m3/demo/scene_040.png) and
[20-second video](results/examples/m3/demo/demo.mp4) come from a separate geometric-control
loop run, with 100 actual 640×480 AirSim frames. Capture configuration, timestamps, hashes
and run result are stored alongside the media. The body-relative trailing camera changes
only the selected simulator's view. This demo is excluded from comparison timing.

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/capture_demo.py --local-config .local/environment.json --exclusive-simulator --output results/runs/my-demo
```
