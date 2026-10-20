# M1 baseline acceptance

All three numerical baseline implementations completed genuine AirSim hover, smooth 3D
transfer and hold on the dedicated M0-validated Blocks instance. These are development
acceptance runs, not the frozen M3 comparison or a robustness claim. They use the same
task settings, actuator map, feedback provenance and nominal operating envelope.

| Controller | Whole-run time-weighted position RMS (m) | Steady-hold RMS (m) | Saturation | Result |
| --- | ---: | ---: | ---: | --- |
| PID | 0.013419 | 0.002412 | 0 | Passed |
| LQR | 0.005685 | 0.000122 | 0 | Passed |
| Standard SE(3) | 0.006147 | 0.000069 | 0 | Passed |

Each run lasted 12 s of simulation time, with an initial 2 s hover, degree-nine C4
transfer over 4 s by `(0.6,0.4,0.3)` m and a final hold. Hold evaluation excludes the first
second following the transfer. Acceptance threshold is 0.1 m RMS during that hold.
No collisions or allocation saturation occurred. The small initial altitude loss from
motor spin-up is retained and contributes to whole-run error. Different exact control
timestamps arise from observed stepping jitter; metrics integrate squared error over time.

The full control-cadence logs, exact configurations, source revision/diff hash, resolved
vehicle parameters, timings and results are archived losslessly in `results/examples/m1/`.
Large physics-cadence logs remain in ignored `results/runs/tune-*-01/physics.jsonl` on the
AirSim host; they are not claimed as externally uploaded artefacts. To reproduce physics
samples elsewhere, rerun the provided experiment command on the recorded compatible build.
The archive's `tune-*` source names identify development runs; they are not evidence of a
separate train/evaluation split. No gains were selected from a final comparison matrix.

Figures in `results/examples/m1/figures/` regenerate from the full archived control logs:
3D/top-down trajectories, per-axis/Euclidean errors, requested versus telemetry-estimated
wrench, rotor use/saturation, summary and timing. PDF/SVG/PNG are provided with editable
Python source. Times New Roman was unavailable; the recorded fallback is DejaVu Serif.
`geometric/scene.png` is an actual AirSim camera-0 scene capture after the geometric run,
with a separate provenance file; it is not an offline rendering or an upstream screenshot.

Measured wall-loop p95 was roughly 21 ms versus a nominal 12 ms simulation control period,
with all development-run wall deadlines missed. Paused stepping preserves control's
simulation-time interval, so these results do not demonstrate real-time feasibility.
Controller computation excludes gain construction/reset, and is logged separately from
allocation/RPC/total-loop overhead. No unpaused timing run is claimed at M1.

Numerical validation: equilibrium, corrective moment signs, reset, LQR nominal discrete
closed-loop eigenvalues, reference derivative consistency, normalized-direction derivatives,
singular cases, physical transforms/allocation, irregular-time RMS and censored settling.
Run `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest -q`; opt-in read-only AirSim
integration uses `IET_CH5_LOCAL_CONFIG`. Source/licence notices and equation mapping are
in `THIRD_PARTY_NOTICES.md`, `source_manifest.json`, `controller_mapping.md`.

M2 observer/MPC, verified disturbance scenarios and M3 fixed benchmark are still pending.
No successful hardware transfer, aggressive-flight envelope or statistical ranking is claimed.
