# Modular UAV controller case study

The reusable case study separates numerical algorithms, trajectories, experimental conditions and measured evaluation. The original 51-run M3 dataset and earlier circular/figure-eight videos remain unchanged. This extension introduces seven reference configurations and an additional independent 70-run protocol; its [completed measured results](modular_results.md) retain passes, failures and the aggressive MPC repeatability limitation.

## Project map

```text
src/iet_ch5/
  simulation/              SimulationBackend protocol, AirSim implementation, backend factory
  controllers/
    pid/                   controller.py, config.json, README.md
    lqr/                   controller.py, config.json, README.md
    geometric/             controller.py, config.json, README.md
    dob/                   controller.py, config.json, README.md
    mpc/                   controller.py, config.json, README.md
    configuration.py       owned defaults and strict partial JSON overrides
    common.py              shared attitude / derivative algebra
  trajectories/
    configs/               seven SI/NWU JSON reference specifications
    timing.py              C4 transfers, phase-rate entry/cruise/exit
    library.py             p/v/a/jerk/snap and heading derivatives
    analysis.py            reference-only physical feedforward screen
  experiments/
    configuration.py       suite + trajectory + controller resolution
    engine.py              freeze, exclusive endpoint lock, journal, explicit resume
    publication.py         compact results, all-run tables, reproducibility, figures
    media.py               checked genuine AirSim footage and synchronized comparison
  runner.py                one controller loop, feedback, allocation and truth logging
  evaluation/              independent truth resampling and metrics
configs/experiments/       runtime, 70-run matrix, separate robustness matrix, override example
scripts/run_case_study.py  list / plan / run / demo / publish
scripts/plot_case_study.py regenerate figures from compact committed data
```

Legacy controller imports and `backends.airsim_backend` remain compatible. `configs/controllers.json` is the historical snapshot used by earlier experiment commands. The new entry point loads each controller's own config. No numerical gain or controller law was changed by the layout migration.

## Common environment

`SimulationBackend` exposes normalized `State`, verified `VehicleParameters`, measured simulation stepping, wind, image capture and cleanup. The AirSim adapter contains simulator-specific RPC. All methods deliver `WrenchCommand(f,tau)` to the same verified physical allocator. NWU/FLU upward-thrust convention, motor order and bridge validation remain as documented in [model_and_conventions.md](model_and_conventions.md).

The backend can be injected into `runner.run(..., backend=...)`; future backends must implement measured timestamps and the declared physics archival decoding. Only AirSim is currently implemented. No offline replacement is presented as simulator evidence.

## Trajectories and tuning

| ID | Shape | Traversal | Role |
|---|---|---:|---|
| hover | fixed 3D point | 6 s | equilibrium |
| transfer | C4 three-axis displacement | 6 s | transition |
| circle | horizontal closed circle | 8 s | planar curvature |
| figure_eight | horizontal Gerono eight | 10 s | changing curvature |
| helix | circular ascent, final altitude hold | 10 s | climb coupling |
| spatial_course | 3D multi-harmonic eight, varying yaw | 12 s | nominal spatial tracking |
| agile_course | 6 m wide 3D multi-harmonic eight, varying yaw | 8.5 s | aggressive stress |

Each configuration adds 1 s initial and 1 s final hold. Phase-rate ramps have zero derivatives at joins; references provide analytic derivatives through snap. Constant phase rate is not constant Cartesian speed. Helix ends at a different altitude; final hold follows its actual endpoint.

Copy any trajectory JSON to a custom file and pass its path to `--trajectory`. Tune `timing`, `shape`, `heading`, `evaluation`, `safety` and camera position in SI units. `analysis.screen` checks ideal static feedforward rotor coupling against the selected vehicle, including angular acceleration moments. It does not prove closed-loop feasibility or account for motor lag/obstacles.

The first 8 s / 1.25 s ramp aggressive candidate demanded 1.042 motor input and was rejected before evaluation. After reference-only screening, the frozen 8.5 s / 1.75 s ramp candidate peaks near 4.02 m/s and needs at most 0.882 ideal motor input. A separate geometric-controller AirSim pilot completed successfully before freezing the matrix. [Measured pilot and rejected/reference-only candidates](../results/examples/modular_preflight/README.md) retain the underlying evidence. The candidate was adjusted for physical authority; controller limits remain unchanged.

Controller tuning: change a method's `config.json` for a new baseline, or supply a partial map with `--controller-overrides`. See `configs/experiments/tuning.example.json` and each method's parameter table. That example deliberately changes PID gains and disables DOB; it is not used in the published matrix. For a matched DOB ablation use only `{"dob":{"enabled":false}}`, retaining the same nominal geometric gains. Positive gain/limit/weight validation rejects invalid arrays, unknown keys and nonfinite entries.

## Execution

Install and launch the isolated simulator using the root README. Commands below assume an owned, capability-checked endpoint in ignored `.local/environment.json`.

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
.venv/bin/python scripts/run_case_study.py list
.venv/bin/python scripts/run_case_study.py plan --output .local/case-study-plan.json
.venv/bin/python scripts/run_case_study.py run --controller geometric --trajectory agile_course --repeats 1 --local-config .local/environment.json --exclusive-simulator --output results/runs/my-pilot
.venv/bin/python scripts/run_case_study.py run --local-config .local/environment.json --exclusive-simulator --output results/runs/my-case-study
.venv/bin/python scripts/run_case_study.py publish --output results/runs/my-case-study --destination results/runs/my-publication
.venv/bin/python scripts/plot_case_study.py results/runs/my-publication --output results/runs/my-regenerated-figures
.venv/bin/python scripts/run_case_study.py demo --controller all --trajectory agile_course --local-config .local/environment.json --exclusive-simulator --output results/runs/my-agile-demo
.venv/bin/python scripts/run_case_study.py publish --output results/runs/my-agile-demo --destination results/runs/my-agile-video
```

`--controller` and `--trajectory` accept several IDs, `all`, or a trajectory JSON path. Demo uses one repeat unless specified; video publication requires one trajectory and one run per method. Record separate demo groups for additional trajectories. `--suite configs/experiments/robustness.json` selects separate wind, model-mismatch and five paired noise/delay seeds. This is an executable additional protocol; it is not automatically claimed as new measured evidence. The original M3 dataset provides the earlier disturbance/mismatch/noise comparisons.

Use a new output directory. `--resume` checks resolved configuration and executable/configuration content fingerprint, skips completed passes and failures alike, and archives incomplete attempts before rerunning them. Source/config changes require a new experiment. Runtime files retain full metadata, simulator provenance, original results and failure traceback. Progress is written atomically after each run. A completed tracking failure is a valid experiment outcome; the batch command exits unsuccessfully for infrastructure/configuration failures, while per-run tracking acceptance remains explicit in the journal and report. An ignored endpoint-specific lock prevents two instances of the new CLI from commanding the same simulator concurrently.

## Evaluation and interpretation

The matrix evaluates five frozen methods on seven trajectories twice (70 runs). Independent simulator physics truth is interpolated onto the common 12 ms grid, including terminal endpoints. Traversal RMS, full duration/per-axis error, velocity/heading/attitude/rate errors, settling/recovery, command/telemetry-estimated wrench, motor range/saturation, allocation residuals, limits, solver statuses/fallback and wall/controller timing are retained. Two repeats check a predeclared 0.02 m maximum trajectory difference; they are not statistical evidence of superiority.

Nominal acceptance uses traversal RMS <0.10 m; aggressive acceptance <0.15 m. Evaluation speed/tilt limits are 2 m/s / 0.30 rad nominal and 5 m/s / 0.85 rad aggressive. Abort limits are separate, shared across methods: 3.5 m tracking error, 7 m/s speed, 1.1 rad tilt or collision. An aborted run's RMS is explicitly limited to available data and cannot count as a pass or a complete-window comparison. Limits and failures stay in the tables.

MPC retains its 2 m/s and 0.3 rad **prediction** envelope (conservative component boxes) in every task. Aggressive reference feasibility for the physical aircraft does not imply feasibility of this near-hover QP. PID acceleration/rate/velocity clipping also remains active. Large tracking errors, QP infeasibility/fallback or divergence on stress tasks are meaningful case-study outcomes. The suite-wide aggressive evaluation limits do not alter a controller's internal constraints.

Plots show complete failed runs and terminate actual data at aborts. Native footage is separately acquired; image hashes, fixed camera pose and exact paused image/truth timestamps are verified before MP4 encoding. MP4 uses 1x simulation time, GIF 2x; any aborted panel explicitly holds its last real image. Capture is outside control timing and is excluded from benchmark comparisons. Raw logs/images stay local; compact NPZ/configuration/results and raw hashes are published. Paused simulation does not establish real-time operation or hardware transfer.
