# M0 environment and bridge report

Recorded 20 October 2026. **Motor/timing smoke acceptance passed on the dedicated
Blocks instance. No controller benchmark has yet passed in this milestone.**

This is the historical M0 report. Later long-run testing exposed wall-time stepping and
motor-registration issues; the final measured-simulation-deadline bridge, retained failures
and corrected ring-fill check are documented in [bridge_regression.md](bridge_regression.md).
The completed five-controller matrix is in [case_study.md](case_study.md).

## Environment and isolation

The repository was clean at `61a8ca487e616f9f992aa4c309aecf8cb7f0d4bc` on main.
Implementation uses `feat/ch5-airsim-controllers` and `src/iet_ch5/`.
The host already had a Blocks process with another project's settings. That process
was neither reset, armed, paused nor commanded. A second instance reused the installed
Blocks scene and plugin with an independently selected RPC port, vehicle `Chapter5`,
settings and log. Local connection/installation paths are in ignored `.local/environment.json`.
No base Python environment was changed, no Unreal rebuild was performed, and all
pre-existing AirSim tracked modifications retain their initial full-diff hash.

* Unreal Engine 4.27.2, Blocks `/Game/FlyingCPP/Maps/FlyingExampleMap`.
* Linux, CPython 3.10.12, isolated `.venv`, uv 0.11.2, dependencies in `uv.lock`.
* NVIDIA GeForce RTX 5060 Ti. Runs here use paused simulation-time stepping;
  they make no real-time feasibility claim.
* AirSim base `13448700ec2b36d6aad7a4e0909bc9daf9d3d931`, 16 pre-existing tracked
  modified files and two untracked scene directories, inventoried in
  `airsim_source_inventory.json`. The telemetry subset is preserved in
  `../patches/airsim-dynamics-telemetry.patch`; it excludes machine-specific build changes.
* Loaded AirSim plugin SHA256:
  `88c360b81842ab5547eecebbe5a1816e89bb254e8ebff94e2b479bbfd7b17030`.
  Its mapping was checked in the dedicated process's loaded modules. Three relevant
  plugin/source header copies match the inspected source. The binary predates this task;
  an original build receipt is unavailable. Runtime capability/sign tests establish
  observed compatibility, not a claim of independently reproducible binary compilation.

The official Python client contains local extensions. The chapter bridge uses an original
bounded MessagePack-RPC transport with explicit request IDs and a separate motor socket,
avoiding a machine-wide change to AirSim's legacy Python dependencies. It talks to the same
installed RPC methods; its own source revision and dependency versions must accompany runs.

## Runtime capabilities and active parameters

The actual server reports version 1, minimum required client version 1. Successful calls
include `getDynamicsParameters`, `getDynamicsStateBatch`, extended `getRotorStates`,
truth kinematics/environment, pause/continue and motor passthrough. See
`../results/examples/m0/capabilities.json` for actual responses and RPC timing.
Missing parameter access fails explicitly; there is no generic-vehicle fallback.

| Quantity | Observed value |
| --- | --- |
| Vehicle | Chapter5 / SimpleFlight quad-X |
| Mass | 1.0 kg |
| FLU inertia diagonal | 0.0067211874, 0.0080406871, 0.0142788757 kg m² |
| Gravity | 9.8066502 m/s² |
| Rotor horizontal offsets | ±0.16086680 m |
| Maximum rotor reference thrust | 4.1794453 N |
| Maximum rotor reaction coefficient | 0.05556217 N m |
| Active air density | 1.2103494 kg/m³ |
| Motor filter time constant | 0.005 s |
| Physics timestamp increment | 0.003000064 s |
| Predicted equal hover input | 0.59380585 per rotor |

Full parameters, allocation matrix and raw physics samples are in
`../results/examples/m0/bridge_smoke.json`. Truth is the feedback source; SimpleFlight's
estimator is a ground-truth pass-through, so this is not estimator validation.

## Actual tests and failures

The smoke script resets and initializes **only the explicitly designated exclusive
instance**, placing it at NWU `(0,0,2)` with zero initial velocity. This is setup before
evaluation, not disturbance injection. The scene's configured spawn Z does not establish
the RPC-relative initial altitude, so setup verifies and controls it explicitly.

1. Equal-input hover: estimated steady thrust matches mg and rate remains zero.
   Small initialization drop is retained in the raw evidence; no position-hold claim is made.
2. Separate +0.01 N m roll/pitch/yaw requests: positive FLU body-rate changes in every axis,
   confirming geometry order and reaction signs. Rotor thrust matches filtered input times
   its reference coefficient to floating-point tolerance.
3. Physics timestamps advance in approximately 3 ms steps; samples have strictly increasing
   sequences/timestamps and do not advance while paused. Measured 12 ms continuation calls
   may yield 12 or 15 ms of actual physics, so control uses measured timestamps.
4. Goal persistence: input remains installed after the short motor command duration expires.
   The 5 ms motor filter transient is saved at physics cadence. This does not constitute a
   motor hardware identification experiment.
5. Batch limit/truncation works. Source ring capacity is 10,000. A 30.1 s simulation advance
   causes a stale nonzero cursor to report `samples_dropped=true`; the observed first/last
   cursors and raw non-overflow samples are retained. Reset clears the buffer and changes
   reset_id, while sequence numbering continues. Consumers must reset their cursor.
6. First bridge attempt failed: waiting for a motor future after a 12 ms continuation caused
   a five-second RPC timeout because the worker's command-period wait had not finished.
   The corrected bridge never waits for a future while paused; it drains completed replies,
   bounds the outstanding queue and checks observed input against the requested goal.
7. Initial pytest execution discovered host ROS plugins through inherited Python paths and
   failed on an unrelated missing YAML package. `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` isolates
   this project's tests without modifying the host ROS environment.

Motor-request registration is checked from the final physics sample of each step, and raw
samples expose intermediate input latency. This interface is validated for this installed
build; different builds must rerun M0. Computation/RPC/loop timing will be logged separately
in controller runs. Paused stepping is not evidence of unpaused deadline performance.

## Source selection

Read the pinned FDCL C++ standard SE(3) implementation and safe-control-gym PID/LQR/linear
MPC implementations, with licences retained and revisions in `../THIRD_PARTY_NOTICES.md`.
FDCL's Python files at the inspected revision model a dumbbell spacecraft; they are not a
drop-in quadrotor controller. Prefer a small documented numerical implementation over
importing these projects' simulation stacks. The observer still requires primary-literature
selection in M2. TinyMPC and RPG MPC remain candidates, not selected/copied components.

## Reproduction and remaining limitations

Use the English README commands for installation, isolated launch, read-only capability
inspection and explicit motor smoke tests. Integration tests are opt-in, and skip clearly
without a selected AirSim server. No physical-vehicle operation is implemented.

M0 passed the observed bridge acceptance on the recorded binary. A fresh build recipe and
exact force/moment injection are not yet verified. Controllers, benchmark matrix, plots,
screenshots/video and case-study conclusions belong to subsequent milestones; no ranking
is supported by these short bridge tests.
