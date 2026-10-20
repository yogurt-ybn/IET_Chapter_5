# Chapter 5 AirSim controller implementation requirements

Date: 20 October 2026. Target repository: https://github.com/yogurt-ybn/IET_Chapter_5

## Objective and scope

Implement the conventional quadrotor controllers in Chapter 5.3 and deliver a reproducible AirSim case study for Section 5.3.6. The deliverable is runnable, readable teaching code together with measured comparison data, plots, actual simulator images and deployment instructions.

The five required implementations are cascaded PID, near-hover state feedback/LQR, geometric SE(3) control, disturbance-observer compensation around a fixed nominal controller, and constrained MPC. A family name does not require implementing every variant discussed in the chapter. Learning-based controllers belong to a later task. Preserve a backend-independent interface for future sim-to-real work; real flight and claims of successful hardware transfer are outside this implementation task.

Current status: requirements and source screening only. No controller in this repository has yet been implemented or validated by this task. Existing machine-specific AirSim changes and historical logs are not evidence that these chapter benchmarks have passed.

## 1. Start with the existing environment

Inspect local instructions, Git status, simulator processes, AirSim client/server revisions, Unreal project, Python environments and settings before changing anything. Reuse the installed AirSim and scenes. Preserve existing uncommitted source modifications, settings, scripts and data. Create an isolated environment for this repository if needed; do not upgrade the machine's base environment or rebuild Unreal unnecessarily.

The initial inspection found an AirSim source checkout based on commit `13448700ec2b36d6aad7a4e0909bc9daf9d3d931`, with local modifications. The local Python client includes `getDynamicsParameters`, `getDynamicsStateBatch`, and extended rotor telemetry. Those are local extensions, not standard upstream API guarantees. Detect capabilities at runtime, verify that the running server supports them and record the source diff and exact build actually used. A listening RPC port does not prove motor-level or extension compatibility.

Produce `docs/environment_report.md` with versions, scene and launch procedure, vehicle name/type, server capabilities, timing observations and verified parameters. Keep personal host addresses, credentials and machine-specific absolute paths out of public example configurations. Put local paths in ignored local configuration or an untracked handoff note.

Do not reset, arm, change settings or send commands to an active simulator until checking whether another experiment is using it. Use a separate scene/process/port where practical. Restrict all vehicle actions to simulation.

## 2. Shared physical and software contract

The chapter, especially Sections 5.2 and 5.3, is the source of truth. Use SI units. In the chapter convention, inertial e3 points upwards, the body third axis points along positive rotor thrust, and R maps body coordinates into inertial coordinates:

```text
p_dot = v
m v_dot = f R e3 - m g e3 + d_f
R_dot = R [omega]_cross
J omega_dot = tau - omega cross (J omega) + d_tau
u = [f, tau_x, tau_y, tau_z]
```

Here f is scalar collective thrust [N], tau is a body moment [N m], d_f is an inertial force and d_tau a body moment. Euclidean tracking errors are desired minus actual. LQR may use actual-minus-reference state deviations with explicitly documented negative feedback. Use W for the LQR input weight, R for rotation and J for inertia. Distinguish the geometric controller's inertial force vector from scalar collective thrust.

Create `docs/model_and_conventions.md` covering state order, units, errors, quaternion order, all coordinate transforms, rotor numbering/spin directions, allocation signs, controller models and departures from the chapter. Preserve native math semantics: a text extraction of the manuscript may omit dots, hats, square roots, fractions or subscripts.

Choose and document a complete right-handed upward inertial frame and upward-thrust body frame. For example, NWU/FLU can be mapped from AirSim NED/FRD using proper rotations on both world and body bases. Verify the installed API's angular-rate frame rather than assuming it. A z-only sign flip is insufficient. Test position, velocity, attitude, yaw, body rate, force, moment and inertia transformations, including nonzero attitudes and round trips.

Provide typed records for `State`, `Reference`, `VehicleParameters`, `WrenchCommand` and `RunResult`. The controller contract should be equivalent to:

```python
controller.reset(initial_state, reference, parameters)
wrench, diagnostics = controller.update(state, reference, dt)
```

State contains p, v, R and body omega, with a timestamp and declared feedback provenance. Reference contains position, velocity, acceleration, heading and the additional derivatives/preview required by each method. Controller memory is reset for every run. Adapters, allocation, logging and plotting must not be duplicated in each controller.

All five controllers deliver the same thrust/moment request to one allocation/actuator module. Main comparison runs use verified motor-level access such as `moveByMotorPWMsAsync`. AirSim's built-in position/velocity/path commands are permitted for clearly separated setup, diagnostics or built-in-autopilot demonstrations. Relabelling those calls as PID/LQR/SE(3)/MPC does not implement the chapter algorithms.

Derive the mixer from the active vehicle geometry and reaction moments. Record the conversion from physical rotor force or squared speed to normalized motor command, input ordering and motor response/lag. Do not assume that normalized PWM is rotor speed, or blindly square it. The installed AirSim source and verified runtime parameters determine the mapping. Include saturation, residual wrench diagnostics and justified anti-windup. If an achieved input is estimated rather than measured, label it accordingly.

## 3. Controller deliverables

| Controller ID | Minimum implementation | Required explanation/validation |
| --- | --- | --- |
| `pid` | Position/velocity outer loops, thrust/attitude conversion, attitude and rate feedback to moments, filtering and anti-windup | Identify each loop's rate, gain, limit and feedback. Demonstrate 3D hover and smooth transfer. |
| `lqr` | A documented discrete-time near-hover coupled tracking model with altitude/yaw channels, reference/feedforward and offline gain calculation | Publish state/input ordering, A/B, Q/W, equilibrium and design interval. The chapter's planar pitch example alone is not a complete aircraft controller. |
| `geometric` | Translation feedback plus SO(3) attitude and angular-rate tracking, with thrust direction and magnitude consistent with the chapter | State the source variant and frame/error sign adaptations. Handle thrust-direction degeneracy, heading singularity and derivative requirements explicitly. |
| `dob` | The same fixed nominal controller as an ablation baseline, with a fully specified force and/or moment disturbance observer and compensation | Compare enabled and disabled compensation with identical nominal gains. Use achieved/estimated applied input in observer residuals. Document observer bandwidth, filtering, bounds and the uncertainty actually addressed. |
| `mpc` | A real constrained receding-horizon optimisation that produces the shared wrench command | Publish prediction model, discretisation, horizon, cost, constraints, solver, warm start, tolerances and fallback. A near-hover linear MPC is acceptable and must be labelled as such. Do not claim a nonlinear formulation was implemented when it was not. |

For MPC, rotor feasibility couples thrust and moments. Use the allocation map to express feasible inputs; separate per-channel wrench clipping is not an equivalent feasible set. State/tilt/speed constraints must have a consistent prediction model. Record infeasibility, solver timeouts and fallback activation. Terminal choices must be stated; do not assert stability or recursive feasibility without their assumptions.

For disturbance rejection, identify a realizable observer rather than only adding a placeholder disturbance input. A translational force estimate affects the commanded force vector and desired attitude; it cannot simply be subtracted from a body-moment command. A moment-only observer must be labelled with that narrower scope. A full 3D sliding-mode controller can be an optional extension after the five core implementations; neither SMC nor H-infinity is required merely because the chapter discusses them.

## 4. Verify the AirSim bridge before benchmarking

1. Confirm RPC/client/server versions and motor API support with bounded timeouts.
2. Obtain active vehicle parameters. If the custom parameter API is unavailable, derive parameters from the exact active source/configuration and validate them; never silently fall back to generic values.
3. Verify hover thrust and individually verify the signs of roll, pitch and yaw moments with small perturbations, including rotor order. Record the test configuration and evidence.
4. Verify state timestamps, physics-step sampling, command persistence, reset/arm behaviour and actuator lag. Inspect duplicate, stale, skipped and out-of-order samples. The custom batch API is useful only after its sequence/overflow semantics are checked.
5. Establish a simulation-time control loop. Characterise pause/continue or frame stepping with the installed build, measured dt and asynchronous command ordering. Do not wait for an asynchronous command to finish while the simulator is paused if that prevents progress.
6. Record control computation, RPC/allocation overhead, total wall-clock loop time, simulator dt and deadline misses separately. A paused benchmark is not a demonstration of real-time feasibility. Use a separate unpaused timing run when claiming real-time operation.

Provide focused numerical tests for coordinate transforms, rotations, mixer signs, hover equilibrium, bounds, reset and metrics, plus a small AirSim integration smoke test. Missing AirSim must skip integration tests explicitly rather than report a successful simulator run. Use the same verified bridge and actuator mapping in every comparison.

## 5. Common experiments and measured outputs

Use one quadrotor and the same initial state, reference, actuator limits, feedback pipeline and evaluation interval across methods. Start with a common near-hover operating envelope; evaluate aggressive flight separately and report each design's limits. Tune on separate nominal manoeuvres and freeze configurations before evaluation. All methods may access the same reference information; report the derivatives and preview each actually uses.

Required experiments:

1. Hover, smooth position transfer, then hold. Separate transfer error from steady-hold error.
2. A smooth horizontal circle/loop at fixed altitude and heading, with a feasible entry/exit trajectory. A figure-eight can be an additional test.
3. Repeat a task with a documented disturbance and compare nominal/observer-enabled operation.
4. Separate plant/controller parameter mismatch and feedback-noise/delay cases before any combined stress test.

For a prescribed force/moment pulse, implement and verify injection at the plant, including frame, amplitude, onset and duration. Changing the requested controller command or teleporting the vehicle is not external-force injection. AirSim wind velocity can provide a separate wind scenario: record its velocity profile and the aerodynamic load model; do not report wind [m/s] as a force [N]. If exact force injection needs a new server extension, isolate it as a minimal patch, preserve prior modifications and document the build. Until verified, report the corresponding force-pulse test as unavailable, not completed.

First use the same ground-truth state as feedback to isolate control differences. A later experiment may use a shared sensor/estimator pipeline. Always compute evaluation errors from independent simulator truth. In SimpleFlight, the object named `kinematics_estimated` may equal truth: inspect actual provenance instead of claiming estimator validation from its name.

Declare common numerical limits/tolerances in configuration before final evaluation, following bridge validation and feasible-envelope checks. Deterministic runs need an explicit repeatability check. For stochastic cases use at least five paired seeds per condition as an initial protocol, reporting sample count, distribution and failures; this is not a statistical power claim. Store the actual disturbance/noise realization because different simulator builds may not expose complete seed control.

Minimum metrics:

- Position RMS error `sqrt(mean(sum((p_d-p)**2, axis=1)))` and maximum Euclidean position error in metres, using the same valid sample interval. Use regular simulation-time resampling or a documented time-weighted metric when sampling is irregular.
- Per-axis errors, attitude/rate errors with definitions, transfer completion and disturbance recovery. Settle/recovery requires a specified tolerance and dwell condition. Preserve non-recovery as censored/failed rather than zero.
- Thrust/moment histories, requested versus achieved input, rotor saturation fraction/duration, allocation residual, constraint violations and failed runs.
- Controller computation median/p95/max, total loop timing, missed deadlines, MPC solver status and fallback usage.

An effort proxy must have stated normalization/units. Do not label squared command magnitude as electrical energy without a power model. Do not discard failed or divergent runs when aggregating accuracy.

Use a common log schema containing simulation/monotonic timestamps, run/config IDs, reference, truth, feedback, requested wrench, achieved/estimated wrench, motor commands/states, disturbances, constraints and timing. Save exact software revisions, config hashes and seeds with every run.

Produce reference/actual 3D and top-down trajectories, position errors versus time, thrust/moment histories, rotor use/saturation, summary tables and timing comparisons. Mark manoeuvre/disturbance intervals. Save publication plots as editable-source plotting scripts plus PDF/SVG and PNG. Use clear labels, consistent colours and Times New Roman where available for book figures. Include actual AirSim screenshots and one short demo video when capture is available. Identify the run/controller/config and avoid presenting an official screenshot or a synthetic offline simulation as evidence of an AirSim benchmark.

## 6. Reuse existing implementations carefully

Prioritise original author/laboratory repositories and mature solvers. Read the actual implementation before selecting it. Candidate sources are listed in `docs/UPSTREAM_CANDIDATES.md`; these are screening leads, not validated drop-in AirSim integrations.

For every reused component record URL, pinned commit, original files, authors, licence, corresponding paper, copied/adapted portions, coordinate/input changes and tests in `THIRD_PARTY_NOTICES.md` and a machine-readable source manifest. Retain required licence notices. A paper citation alone does not license source code. If a suitable implementation cannot be reused, document a small original implementation from cited equations and test it. Do not claim that a repository's published results were reproduced merely because its code was consulted.

Do not import whole ROS/PyBullet/Gazebo stacks solely to reuse a small controller. Prefer a clear numerical core and an AirSim adapter. Do not assign the user's original work a new project licence without recording that decision; upstream notices remain mandatory.

## 7. Repository layout and execution

Suggested structure (adapt where useful without duplicating responsibilities):

```text
src/iet_ch5/{types,frames,allocation,runner,logging}.py
src/iet_ch5/backends/airsim_backend.py
src/iet_ch5/controllers/{pid,lqr,geometric,dob,mpc}.py
src/iet_ch5/references/
src/iet_ch5/evaluation/
configs/{vehicle,controllers,tasks,experiments}/
scripts/{check_environment,run_experiment,run_benchmark,plot_results}.py
tests/{unit,integration}/
docs/{environment_report,model_and_conventions,controller_mapping,experiment_protocol}.md
results/examples/
THIRD_PARTY_NOTICES.md
```

Provide a pinned environment and English README with installation, scene startup, capability check, single-controller execution, full comparison and plot regeneration commands. Commands must be tested before being advertised as working. CI runs unit tests without Unreal; integration tests are opt-in on the AirSim host. Keep full simulator installations, proprietary scene assets, credentials and large/generated logs out of Git. Commit compact genuine example logs/results with provenance; store larger artefacts through documented external/release storage if needed.

Work in a dedicated repository directory and feature branch. Preserve existing local AirSim work. Commit/push coherent implementation milestones to the user-designated repository and open PRs against main for review. Do not force-push or merge unfinished comparisons. Report branch, commit, PR, actual commands executed, results and limitations at each handoff.

## 8. Execution order and acceptance

### M0: Environment and bridge

Deliver environment/source-diff inventory, source selection, model/convention contract, verified runtime parameter access and motor/timing smoke-test evidence. This checkpoint must pass before quantitative controller rankings. If blocked, report the exact missing capability and preserve useful diagnostics.

### M1: Three baseline controllers

Deliver the common infrastructure plus PID, LQR and geometric control. Each must complete a genuine AirSim hover/transfer run with saved logs and regenerated plots, or clearly report a failed acceptance item. Add the chapter-equation-to-code mapping and record any extension beyond the chapter's illustrative planar equations.

### M2: Disturbance compensation and MPC

Deliver the observer with an enabled/disabled ablation, a functioning constrained MPC, verified disturbance mechanism, constraints and failure handling. Exercise solver failure and actuator saturation explicitly.

### M3: Reproducible case study

Run the fixed common benchmark matrix. Commit compact result examples, comparison tables, publication plots, actual scene images, commands/configuration and an English case-study interpretation based on the measured data. Distinguish successful/failed/unavailable tests and nominal/stress conditions. Preserve a concise list of remaining sim-to-real requirements for Section 5.5 without claiming physical deployment.

Final acceptance requires all five controller implementations, shared verified motor interface, actual AirSim evidence, documented provenance, reproducible experiment/plot commands and honest failure reporting. Do not mark M3 complete based only on unit tests, offline rollouts, source inspection or attractive plots.