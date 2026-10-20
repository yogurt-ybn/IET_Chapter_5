# Implementation handoff

Branch: `feat/ch5-airsim-controllers`. Review: https://github.com/yogurt-ybn/IET_Chapter_5/pull/2
(draft, not merged). The numerical benchmark source is `2e6d9ed`; publication analysis,
case-study text and capture tooling are anchored at `a878670`, followed by the verified
media/handoff commit. Use Git HEAD for the final delivery revision.

## Delivered milestones

* **M0:** installed environment/source inventory, retained telemetry patch/licences,
  NWU/FLU contract, physical allocation, actual hover/moment/persistence/lag/pause/batch/
  overflow checks. Long-run failures led to an acknowledged-goal, measured-simulation-time
  bridge; both abandoned attempts remain available for audit.
* **M1:** PID, coupled 12-state LQR and standard SE(3), with genuine AirSim transfer logs,
  reset/sign/model tests, equation mapping and regenerated figures.
* **M2:** bounded force-only observer with identical nominal gains/off ablation, constrained
  OSQP linear MPC, verified wind mechanism, actual actuator saturation/solver-timeout
  fallback and numerical active-constraint/infeasibility tests.
* **M3 selected case study:** fixed 51-run matrix, 652 s of evaluated simulation time,
  47 tracking passes and four retained mismatch failures. All runs finish their intervals;
  no final bridge/collision/divergence abort. Common-window metrics, all-run records,
  paired seeds, plots, English interpretation and actual scene/video evidence are provided.

All 25 noisy/delayed-feedback runs pass. The matched wind ablation gives 0.117113 m off
versus 0.064075 m on wind-interval RMS. The nominal repeatability checks pass for all five
methods. No final-matrix actuator saturation/tilt/speed violation occurs; separate diagnostics
exercise constraints and failures explicitly. These observations do not establish a
general ranking or real-time feasibility.

## Executed validation

The tested commands and flags are in README.md. Actual work included:

```bash
uv sync --python 3.10 --extra dev --extra plots --extra mpc --locked
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 IET_CH5_LOCAL_CONFIG=.local/environment.json OPENBLAS_NUM_THREADS=1 .venv/bin/pytest -q
.venv/bin/ruff check src scripts tests
.venv/bin/python scripts/launch_simulator.py --local-config .local/environment.json
.venv/bin/python scripts/check_environment.py --local-config .local/environment.json --output .local/launcher-capabilities.json
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/bridge_smoke.py --local-config .local/environment.json --exclusive-simulator --check-overflow --output .local/final-bridge-smoke-sequence.json
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/run_benchmark.py --local-config .local/environment.json --exclusive-simulator --output results/runs/frozen-m3-final-20261020
.venv/bin/python scripts/summarize_benchmark.py --source results/runs/frozen-m3-final-20261020 --output results/examples/m3
.venv/bin/python scripts/plot_benchmark_summary.py --source results/examples/m3 --output results/examples/m3/figures/matrix
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/capture_demo.py --local-config .local/environment.json --exclusive-simulator --output results/runs/demo-geometric-circle
```

Current suite: **22 passed**, including the explicitly selected read-only AirSim integration.
Without local simulator selection: 21 passed, one integration skipped. A clean detached
checkout of the frozen numerical revision was independently installed/tested with Python
3.10: 19 passed, one integration skipped. Its smaller count predates two analysis/constraint
tests. Lint and Git whitespace checks pass. Seven representative control logs are archived
losslessly, every final run has public configuration/result/metric samples, and plots are
regenerated from those data. The separate demo passes and its video decodes to 100 frames,
640×480, 5 fps, 20 s. Capture is excluded from comparison timing.

## Preserved environment and storage

The original AirSim 16-file source diff still has its initial SHA256. No user scene, source
modification or historical experiment was reset or overwritten. The pre-existing simulator
was left running. The chapter-owned process was stopped after its launch procedure and
capabilities were revalidated. Only ignored local configuration contains installation paths
and the local endpoint; use the README to start a fresh dedicated instance.

The complete final original matrix archive is retained in ignored `.local/artifacts/`:
`ch5-m3-frozen-2e6d9ed.tar.gz`, 115,308,253 bytes, SHA256
`2411366d79e84e8e4b47f9c65a563e16ed6fc14b52e085d5995f1af9e7600af3`.
No external archive upload is claimed. The committed compact evidence and all-run metric
samples support reviewing and regenerating the published comparisons. Keep the large raw
archive out of Git; an external/release copy can be made from this verified local artefact.

## Explicit limits

The chosen disturbance experiment is verified AirSim wind velocity with its simplified
drag model. Prescribed external force/moment pulses remain unavailable pending an isolated
server injection patch/build and plant validation. Original plugin build receipt is
unavailable; loaded binary/source hashes and runtime capabilities are recorded. Paused
wall deadlines are missed, so an unpaused timing run is required before a real-time claim.
Feedback noise/delay is synthetic around simulator truth, not a validated sensor estimator.
Physical flight and successful hardware transfer are outside this work. Section 5.5
remaining requirements are listed in case_study.md.
