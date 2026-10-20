# Frozen experiment protocol

M0 uses the verified loaded plugin and telemetry patch described in environment_report.md.
M1 development runs and M2 observer/wind/solver checks precede this matrix. Controller gains
are fixed in configs/controllers.json and shared task/feedback settings in configs/tasks.json.
The benchmark script writes a configuration/matrix/source hash before running anything;
resume rejects any change. No gains may be selected from the final evaluation results.

## Common conditions

One SimpleFlight quad-X in Blocks, shared ground-truth feedback for nominal experiments,
same initial NWU `(0,0,2)` position, identity attitude, zero setup rates/velocities, geometry,
actuator limits, timing bridge and evaluation rules. Setup teleports only before evaluation.
Eight open-loop hover steps prime the motor interface outside evaluation. While paused,
setup then restores and reads back the identical initial position, identity attitude and
zero velocities; controller reset uses this verified truth state and the last physics timestamp. Identical setup does not imply bitwise identical sample timing: M0
observed 3 ms physics with 12/15 ms control continuations. All actual samples/timestamps are
retained, measured dt drives the loop, and two nominal repetitions quantify repeatability.

All controllers have the same analytic planned reference available; PID uses p/v/a and
heading/rate, LQR adds jerk/snap, SE(3)/DOB uses through snap, and MPC uses 30-step preview.
MPC prediction/optimization period is 36 ms, with held wrench between updates, while inner
control feedback and shared allocation run nominally at 12 ms. PID outer period is 36 ms.
Published timing distinguishes optimizer updates from cheap held-output updates.

## Matrix (51 runs)

For each of PID, LQR, SE(3), DOB enabled, and linear MPC:

* Two 12 s hover/transfer/hold repetitions: 2 s initial hover, 4 s C4 transfer by
  `(0.6,0.4,0.3)` m and final hold. Transfer and steady hold are evaluated separately;
  steady hold begins 1 s after transfer ends.
* One 20 s horizontal loop: 0.5 m radius at fixed altitude and zero heading, beginning
  at the common initial position. A degree-nine progression completes one full turn
  between 2–16 s, with velocity through snap zero at entry/exit. This tests a smooth loop,
  not a constant-speed steady circle.
* One wind task repeating the transfer: world NWU wind `[8,-4,0]` m/s on `[7,9)` s,
  zero otherwise. Actual command-change time is logged. A sixth wind run disables DOB
  compensation with exactly the same nominal gains and observer still executing.
* One controller-model mismatch task, with mass +10% and inertia +15% in controller design
  only. The actual plant and verified motor adapter retain their measured values. This is
  explicitly a controller-model uncertainty experiment, not a changed plant build.
* Five paired seeds (0–4) for shared delayed/noisy feedback, with no wind or parameter
  mismatch: 24 ms delay, interpolation from independent physics truth and held independent
  Gaussian noise at 12 ms. Standard deviations are position 0.005 m, velocity 0.01 m/s,
  body rotation-vector 0.001 rad and body rate 0.002 rad/s per component. Every noise array
  is saved; corresponding seeds generate identical arrays across methods. Truth evaluation
  remains independent. Five seeds are an initial paired protocol, not a power claim.

## Wind mechanism and unavailable force pulse

M2 explicitly verifies horizontal acceleration changes under wind at fixed motor inputs.
The active FastPhysicsEngine forms relative body velocity from world `(v-wind)` plus
`omega cross face_position`, selects faces with positive normal velocity, and adds
`-normal * drag_factor * density * normal_velocity²` and the induced face moment.
`MultiRotorPhysicsBody::createDragVertices` computes exposed body/propeller areas and
`drag_factor = area * linear_drag_coefficient/2`. This wind response depends on attitude,
motion and drag geometry. **Wind velocity is not a prescribed force in newtons.**
The installed RPC has no arbitrary plant external-wrench injection. Exact force/moment
pulses are unavailable in this build and are not represented by command offsets, teleports
or equivalent-force claims. No source rebuild that could disturb the user's installed
experimental stack is performed for this optional scenario.

## Metrics and acceptance

Irregular samples use trapezoidal integration: position RMS is
`sqrt(integral(||p_d-p_truth||² dt)/valid_duration)` and max is maximum sampled Euclidean
error. Report per-axis RMS, desired-attitude-to-truth geodesic rotation error and desired
body-rate-to-truth error; for linear controllers these use their near-hover reference tilt.
All are defined in model_and_conventions.md/controller_mapping.md. Settling/recovery needs
Euclidean error <=0.1 m continuously for 1 s. Missing completion is null (censored), never 0.
Recovery is only defined for the disturbance task. Whole-run, transfer and hold errors are
kept distinct; wind interval error is also reported.

Common nominal hold/loop acceptance is RMS <0.1 m. Stress results retain the same threshold
and may legitimately fail. Abort on sampled position error >3 m, tilt >0.7 rad, speed >4 m/s
or collision; partial logs and failures remain in the summary. Additional evaluated limits
are tilt <=0.3 rad and speed <=2 m/s; report violations and duration, not only aborts.
No aggressive-flight conclusion is drawn from this near-hover envelope.

Log requested wrench, steady allocation prediction, filtered-motor telemetry estimate,
residual per physical channel, motor use/saturation, computation, allocation, RPC/stepping,
total wall loop, deadlines, MPC statuses/solve times/fallbacks and source/config/seed.
Mean normalized squared motor command is a dimensionless actuator-use proxy,
`integral(mean(u_i²) dt)/duration`, **not electrical energy**. A residual Euclidean diagnostic
mixes thrust/moment units, so per-channel values are also provided; it is not an effort cost.

## Timing and media

Paused comparisons describe simulation-time tracking, not real-time feasibility. Wall-clock
misses are retained even though pausing protects the physics interval. A future unpaused
test is needed before a real-time claim. Set BLAS/OpenMP threads to one and record this in
the manifest. Capturing images adds overhead and occurs in a separate demo run, excluded
from comparison timing. Frames and video are actual AirSim images with run metadata.

## Reproduction and artefacts

`scripts/run_benchmark.py` runs sequentially on the selected exclusive simulator, retains
all failures and writes an incremental summary. `scripts/plot_results.py` accepts raw or
losslessly compressed logs and creates PDF/SVG/PNG plus summary/provenance. Full benchmark
logs/noise/physics are retained under ignored `results/runs/`; compact genuine examples and
the all-run summary/manifest are committed. The local larger archive is not claimed as an
external upload; external/release storage remains a documented handoff if needed.
No failed runs are removed from summary distributions. Report sample count, median/range
and paired repeatability separately, without unsupported statistical significance claims.
