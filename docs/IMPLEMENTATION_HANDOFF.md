# Implementation handoff

Branch: `feat/ch5-airsim-controllers`. Review: https://github.com/yogurt-ybn/IET_Chapter_5/pull/2
(draft, not merged). The numerical benchmark source is `2e6d9ed`; publication analysis,
case-study text and capture tooling are anchored at `a878670`, followed by the verified
media/handoff commit. Use Git HEAD for the final delivery revision.

## Current demo: figure-eight tracking

The primary demo is `results/examples/figure_eight_demo/comparison.mp4` and its compact
2× GIF preview. Five genuine runs use the same smooth 4×2 m figure eight, with 2 s initial
hold, 28 s traversal through two centre crossings, then 2 s final hold. All use identical
clean source `3eb6692` and unchanged controller gains/plant/motor bridge. All five pass;
no collision, saturation, evaluated state-constraint violation or MPC fallback occurs.
The original 51-run matrix and previous circle media are preserved. See
`docs/figure_eight_demo.md` for the equation, component-bound design check, retained
planning abort, measured table and tested reproduction commands.

Validation: 26 tests including read-only AirSim integration pass; lint/whitespace pass.
All 1,605 images have exact paused-truth timestamps and common fixed camera poses. MP4:
320 fully decoded frames, 1600×1008, 10 fps, 32 s. GIF: 160 frames, 960×600, 16 s at 2×.
The frame-selection age is at most 0.1 s. Original source/binary hashes and the full user
AirSim diff are unchanged. The dedicated figure-eight simulator is stopped after validation;
the original simulator is left untouched. Raw archive remains local:
`ch5-figure-eight-3eb6692.tar.gz`, 457,439,851 bytes, SHA256
`f23652021aaf1c6dcc0f1e41ffd2133591dbbd4a4893213cb87350ed87bbc693`.
Only compact evidence/media are committed; no external archive upload is claimed.

## Earlier circular tracking video

The circular demo is retained at `results/examples/tracking_demo/comparison.mp4` and its 2× GIF
preview. Five additional actual AirSim runs track the same predefined 3 m diameter circle
with fixed heading/local altitude and identical initialization. Each runs for 26 s; all
five pass. The checked fixed external camera and native reference/flown-path markers
replace the original video's visually uninformative body-relative view. See
`docs/trajectory_demo.md` for the reference equation, measured error table, excluded
unaligned preview attempt, commands and limitations. The original 51-run case study is
unchanged. Source/binary hashes and the original AirSim diff were rechecked unchanged.

Validation: 1,305 actual frames have exactly matching image/paused-truth timestamps and
identical camera poses across all five runs. All five sampled reference arrays match.
The MP4 decodes to 260 frames, 1600×1008, 10 fps, 26 s; the GIF has 130 frames over 13 s.
Maximum selected scene age relative to the shared video clock is 0.1 s and is recorded in
the frame-selection timeline. Unit/read-only integration suite: 22 passed. Lint and Git
whitespace checks pass. The new raw archive is retained locally, 370,300,902 bytes,
SHA256 `f662582f65724ecf181e69f071e42e584722678e04d95fcb71b223c941bd1f20`;
only compact evidence/media are committed. The new chapter-owned simulator is stopped
after validation; the original user's simulator remains untouched.

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
