# Chapter 5.4.6 learning-control adaptation contract

Date: 4 November 2026. This task extends the existing Chapter 5 platform; the conventional-controller results and simulator source modifications remain preserved. The supplied adaptation document is private and is not redistributed.

## Primary case study

Compare the **same rigid-body NMPC** with a frozen learned translational residual disabled and enabled. Reuse the AirSim backend, physics feedback, simulation-time stepping and common physical thrust/moment allocator in NWU/FLU coordinates. The earlier linear MPC cannot replace the nominal NMPC baseline.

Prefer a compact Gaussian-process residual workflow informed by the pinned Data-Driven MPC source. Small neural residuals are a fallback if solver integration proves unsuitable. Construct residual targets from simulation-time-aligned measured motion and telemetry-reconstructed applied thrust. Split whole flights, never adjacent samples, between training and independent prediction evaluation. Validate one-step and horizon predictions before deploying a frozen model in closed loop. A fitted model discrepancy is not a measurement of external wind force.

For each on/off pair keep the reference, initial state, model parameters, cost, horizon, discretization, constraints, feedback, allocator and actuator configuration identical. The residual-enable flag is the only controller setting changed. Begin with nominal conditions, then separately evaluate a recorded model-mismatch condition. Declare repetitions, evaluation windows, failures, constraint violations, solver statuses and fallback; retain every attempted run.

Deliver configuration, scripts, dependency/source pins, model/data hashes, raw-log inventories, compact reproducible data, trajectory/error/input plots and a measured interpretation. Paused stepping establishes neither real-time operation nor physical flight.

## Other upstream algorithms

Inspect each supplied fixed commit for original environment, observation/action semantics, dependencies and licence. Classify integration, retraining needs and reference-only use. RL work, if subsequently implemented, must identify retained inner loops. Motor speed, normalized PWM, collective thrust and throttle are different quantities. DATT and Neural-Fly require particular licence scrutiny; publicly accessible source alone does not grant redistribution rights. Audit pretrained weights/data separately.

## Checkpoints

- L0: current source/environment inventory and actual nominal-NMPC AirSim baseline.
- L1: aligned data, flight-level split, fitted GP and independent one-step/horizon evaluation.
- L2: frozen GP inside the existing acados dynamics; matched on/off AirSim comparison.
- L3: public compact artefacts, executable plotting/reproduction commands, upstream adaptation assessment and reviewable GitHub milestones.

Checkpoints require recorded evidence. No completed learning result is claimed by this contract.
