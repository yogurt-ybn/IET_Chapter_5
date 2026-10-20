# AirSim case study: conventional quadrotor control

The five Chapter 5 numerical controllers completed a fixed, genuine AirSim comparison:
**51 runs, 47 tracking passes and four tracking failures**. Every run finished its prescribed
interval; no final-matrix run aborted for bridge failure, collision or divergence. The four
failures are retained and correspond to controller-model mismatch for PID, LQR, standard
SE(3) and linear MPC. DOB compensation passed that condition. This is a near-hover case
study on one simulated vehicle, not a universal controller ranking.

## Experimental basis

The plant is the installed AirSim SimpleFlight quad-X in Unreal Blocks: mass 1 kg,
identified body inertia, rotor geometry and thrust/reaction coefficients, with all methods
using the same verified physical-wrench-to-motor adapter. World NWU/body FLU coordinates
match the chapter's upward-inertial, positive-thrust model. Direct physics truth is the
nominal feedback source. SimpleFlight's object named `kinematics_estimated` is a truth
pass-through, so these experiments do not validate a sensor estimator.

The evaluated numerical source is the clean commit
`2e6d9ed06c3083c17b4a3b074a1537e92bdb0880`. Its frozen manifest is
[`results/examples/m3/manifest.json`](../results/examples/m3/manifest.json), hash
`0d96d8dd39d2ecece5d11b4d7bd15a514b563476daf79b9f846308ac726a5a03`.
Both abandoned attempts and their diagnosed timing failures are preserved in
[the bridge regression record](bridge_regression.md), outside final aggregates.

Before each evaluated run the motors are primed outside evaluation. While paused, setup
restores and reads back the same position `(0,0,2)` m, identity attitude and zero rates and
velocities. Controller memory is then reset. No teleport occurs during evaluation. The
bridge acknowledges the installed motor goal and accumulates actual physics timestamps
until at least the requested 12 ms control interval has elapsed. A single AirSim
`continueForTime` call measures wall time on this build; equating it with physics time was
an experimentally discovered error, corrected before the final matrix.

Gains and limits are fixed, with full models and controller-specific reference use in
[controller_mapping.md](controller_mapping.md), [M2 specifications](m2_acceptance.md) and
[experiment_protocol.md](experiment_protocol.md). PID uses a 36 ms outer cascade, while
attitude/rate control runs at the shared control interval. LQR is a coupled 12-state discrete
near-hover design. Standard SE(3) uses geometric attitude/rate tracking and derivatives
through snap. DOB subtracts a bounded estimated inertial force before computing desired
attitude, with exactly the geometric nominal gains. Linear MPC uses a 36 ms optimization
period, 30-step preview, coupled rotor feasibility and explicit solver failure fallback.

## Common-time analysis

Controller logs contain states at command decisions; their last decision precedes the
requested terminal instant by a slightly different amount in different runs. Final
comparison metrics therefore use the complete independent physics logs, resampled at
12 ms plus the exact terminal endpoint over **0–12 s**, or **0–20 s** for the loop.
Position/velocity/body rates use linear interpolation and attitude uses SLERP. The
redundant zero-time setup snapshot is represented once; original raw files remain intact.
Desired attitude/rate and motor commands are held from recorded control decisions.

Squared errors are integrated with time weights. The summary retains original run metrics
alongside the common-interval metrics; all 51 common-interval outcomes agree with their
original acceptance status. Compact per-run `metric_samples.npz` files allow position/error
metrics and all-run comparison figures to be regenerated without downloading large raw
logs. Failed runs remain in every applicable distribution and table.

## Measured tracking results

Whole-run Euclidean position RMS, metres; nominal transfer uses the median of two runs,
and feedback stress uses the median of five paired seeds. Other conditions have one run
per method. These are distinct task conditions, not pooled accuracy scores.

| Controller | Hover/transfer | Horizontal loop | Wind task | Model mismatch | Noise + delay |
| --- | ---: | ---: | ---: | ---: | ---: |
| PID | 0.010929 | 0.016000 | 0.060945 | 0.207934, failed | 0.012402 |
| LQR | 0.000831 | 0.001291 | 0.045027 | 0.193452, failed | 0.004117 |
| Standard SE(3) | 0.000275 | 0.000152 | 0.062470 | 0.188492, failed | 0.003914 |
| DOB enabled | 0.000269 | 0.000076 | 0.029418 | 0.009158, passed | 0.004013 |
| Linear MPC | 0.003081 | 0.001361 | 0.047271 | 0.203345, failed | 0.005625 |

The transfer is a C4 reference displacement `(0.6,0.4,0.3)` m over 2–6 s, followed by hold.
The loop has radius 0.5 m, fixed altitude/heading, and smooth entry/exit over 2–16 s;
it is not a constant-speed circle. Hold/loop acceptance is RMS <0.1 m, with separate
transfer and hold metrics in the machine-readable summary. Tiny nominal errors reflect
truth feedback, resolved plant parameters and a deliberately mild envelope. Reference
derivative/preview use and selected gains also differ; the table is not an architecture-only
or hardware-performance ranking. No final-matrix rotor saturation or evaluated tilt/speed
violation occurred, so these flights alone do not show an advantage from active MPC limits.

## Disturbance compensation and model uncertainty

The disturbance is **wind velocity** `[8,-4,0]` m/s in NWU over 7–9 s, implemented at the
plant through AirSim's relative-velocity quadratic face-drag model. It is not a prescribed
force pulse in newtons. The actual command profile is saved. The simplified body/propeller
area model is not a validated aerodynamic model of a physical aircraft.

The matched DOB off/on ablation gives wind-interval position RMS **0.117113 m off** and
**0.064075 m on**, about 45% lower in this particular deterministic condition. Sampled
maximum error is 0.176053 m off and 0.081000 m on. With the declared 0.1 m/1 s dwell rule,
the disabled case recovers in 0.744 s after wind removal; the enabled case is already within
tolerance at removal and satisfies dwell, hence recovery is 0 s. Missing recovery would be
null, never substituted with zero. The force-estimate plot is an observer estimate, not
ground-truth applied wind force or load-cell measurement.

The mismatch test changes only controller mass (+10%) and inertia (+15%); the actual plant
and verified allocation retain measured parameters. The four fixed nominal designs retain
a hold offset and fail the common tolerance. The force observer can compensate the lumped
translational mismatch. It does not estimate body disturbance moments, and this result does
not isolate or prove compensation of inertia uncertainty.

For feedback stress, all five methods use the same 24 ms delay and saved held-Gaussian
noise process. All 25 runs pass. The exact arrays for each paired seed are identical across
methods, verified in `noise_pairing.json`; individual values and min/max distributions are
retained. Five seeds are an initial protocol, not a statistical power or significance claim.
Two nominal repetitions per method pass the declared resampled position/reported-RMS
repeatability check; maximum paired trajectory differences are below 0.000012 m. This
repeatability statement applies to the nominal check, not every possible disturbance.

## Constraints, computation and failures

MPC solves a real constrained QP. Separate numerical diagnostics activate predicted tilt,
speed and rotor bounds for an unreachable instantaneous reference; that test is labelled
numerical and uses a longer solver budget. Separate genuine AirSim diagnostics apply an
infeasible collective request to exercise actuator saturation, and force an OSQP timeout
to exercise explicit LQR fallback. Neither is presented as external-force injection.
The 51-run matrix itself has zero MPC fallback activations.

Controller computation, allocation, motor acknowledgement, physics advancement, state RPC
and total loop wall time are recorded separately. For the first nominal MPC run, QP-update
computation median/p95 is approximately 2.90/8.22 ms, including Python preview/vector work;
held-output updates are much cheaper. Its total paused-loop wall p95 is about 27.0 ms.
Across the matrix all 54,193 nominal 12 ms wall deadlines are missed. Pausing preserves
simulation-time feedback while exposing this wall overhead. These results **do not
demonstrate real-time feasibility**; no unpaused deadline claim is made.

## Artefacts and reproduction

* [All-run CSV](../results/examples/m3/summary.csv), [JSON](../results/examples/m3/summary.json),
  per-run configurations/results, metric samples, hashes and paired-noise checks are committed.
* Editable plotting scripts produce PDF/SVG/PNG trajectory, error, wrench, rotor-use,
  saturation, noise-distribution and timing figures. Times New Roman was unavailable;
  the recorded fallback is DejaVu Serif.
* Seven lossless full control-log examples cover all five nominal controllers and DOB
  wind on/off. Full original control/physics/noise files are packaged locally with a
  published checksum in `full_archive.json`; no external upload is claimed.
* [Actual scene image](../results/examples/m3/demo/scene_040.png) and
  [20 s video](../results/examples/m3/demo/demo.mp4) come from a separate successful geometric
  loop run. The video decodes to 100 actual 640×480 frames at 5 fps; relative camera pose,
  original timestamps, source `a878670`, hashes and result are retained. The following
  camera shows the aircraft and scene; no synthetic image supplies simulator evidence.
  Image capture is excluded from comparison timing.

Use the tested README commands. Reproducing the exact numerical source requires the
recorded `2e6d9ed` revision and the M0-compatible installed AirSim extension/build; the
source patch reconstructs all 10 selected extension files, but the original plugin build
receipt is unavailable. A source inspection or an offline rollout is not substituted for
these measured AirSim runs.

## Remaining sim-to-real work for Section 5.5

Prescribed external force/moment pulses are unavailable on this server; they need an
isolated plant-injection patch/build and independent amplitude/frame validation. Further
work includes real actuator/geometry/lag calibration, a common physical sensor estimator,
transport delays and timestamp synchronization, unpaused compute/deadline characterization,
power/energy modelling, hardware-in-loop tests, arming/failsafe/abort validation, and a
separately authorized staged flight programme. No physical deployment or successful
hardware transfer is claimed here.
