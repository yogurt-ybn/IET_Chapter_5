# Rigid-body nonlinear MPC (acados)

This controller is a separate `nmpc` implementation. The historical `mpc` remains the fixed near-hover linear OSQP baseline; previous results are preserved. No AirSim high-level flight command is used.

## Model and interface

State order: **p(3), v(3), q_wxyz(4), body omega(3), telemetry-estimated applied wrench(4)**. Units are SI, world NWU and body FLU. R(q) maps body to world, with upward body thrust. Command is u = [collective N, body moments N m], before the common physical allocator.

```text
p_dot       = v
v_dot       = applied_f / m * R(q)e3 - g e3
q_dot       = 1/2 q ⊗ [0, omega]
omega_dot   = J^-1 (applied_tau - omega × J omega)
applied_u_dot = (commanded_u - applied_u) / measured_motor_time_constant
```

The quaternion is normalized when constructing R and tracking residuals. Its continuous kinematics preserve norm; discretization norm errors are logged. At constant measured allocation/density, the common identical rotor lag maps linearly to wrench lag. The continuous lag prediction remains an approximation to the simulator's discretized filter. The model includes full rotation/thrust-direction and gyroscopic coupling, including non-diagonal inertia. It does not predict aerodynamic drag, changing density, wind loads, collisions or flexible-body effects.

The runner supplies the same available rotor telemetry to the applied-wrench state, prepares the shared reference while paused before evaluation, and logs solver provenance. All methods can access the same analytic trajectory information; this controller uses p/v/a/jerk/snap and heading derivatives to build flatness attitude/rate/wrench targets. A 6 ms reference table is computed before control timing; online interpolation, Python stage updates, solve, diagnostics and solution extraction are included in controller timing. Initialization/build/reference-preparation costs are separately recorded.

## OCP and constraints

Multiple shooting over 40 x 24 ms = 0.96 s. RK4 integration uses four substeps per shooting interval. The nonlinear least-squares residual contains position, velocity, quaternion-relative attitude vector, body rate transformed from the reference frame, applied wrench and commanded wrench errors. The terminal cost uses the first five state residual blocks scaled by `terminal_scale`; it is a heuristic tracking cost, not a proven invariant terminal construction.

Command constraints are exactly `0 <= inverse(allocation) @ command <= 1`, coupling collective and moments. There is **no hard speed or tilt cap inside this OCP**. Shared experiment safety/evaluation limits still apply to all controllers. This design uses physical actuator constraints and tracking penalties rather than a hard state box; it does not guarantee safety, stability, global optimality or recursive feasibility. The same body/plant parameters and shared allocator are used for comparison.

Default acceleration: generated C dynamics/cost/Jacobians, Gauss-Newton SQP-RTI, HPIPM OCP-QP, partial condensing to 10 blocks, shifted primal trajectory and QP warm start. Each control update performs the ordinary RTI preparation and feedback **together**, not asynchronous AS-RTI. Python uses the ctypes wrapper; Cython-wrapper acceleration is not claimed. RTI is one SQP step and does not mean a fully converged nonlinear optimization.

Only status 0 with finite trajectories and a feasible first physical command is accepted. Otherwise an explicitly identified geometric controller supplies the fallback command through the same allocator. Failed solver status, residuals, norm drift, fallback usage and timing remain logged. There is no wall-time abort in the default RTI solve: bounded iteration work supports repeatability, while actual computation/deadline overruns are reported. This is not a real-time guarantee.

## Configuration

| Key | Default | Meaning / units |
|---|---:|---|
| design_dt | 0.024 | Optimization/command-hold period, s; bridge feedback remains 12 ms |
| horizon | 40 | Number of shooting intervals |
| integration_steps | 4 | RK4 substeps per interval; resolve measured motor lag |
| reference_table_period_s | 0.006 | Precomputed analytic-reference spacing, s |
| position_weight | [80,80,120] | Squared position error weights, inverse m² |
| velocity_weight | [8,8,12] | Squared velocity error weights, inverse (m/s)² |
| attitude_weight | [20,20,8] | Relative quaternion-vector residual weights, approximately inverse rad² near zero |
| rate_weight | [.3,.3,.3] | Body-rate error weights, inverse (rad/s)² |
| applied_wrench_weight | [.02,2,2,2] | Applied-wrench residual weights, inverse N² / (N m)² |
| command_wrench_weight | [.02,5,5,5] | Command feedforward residual weights, inverse N² / (N m)² |
| terminal_scale | 5 | Scale of terminal state residual cost |
| solver_type | SQP_RTI | SQP_RTI or SQP; compare work and convergence together |
| condensing_stages | 10 | Condensed horizon; set equal to horizon for uncondensed ablation |
| warm_start | true | Shift previous state/input solution and enable QP warm start |
| max_iter | 20 | Maximum SQP iterations (RTI remains one step) |
| qp_max_iter | 50 | HPIPM iteration cap |
| solver_tolerance | .0001 | Full SQP residual stopping tolerances; RTI does not converge to these each call |
| constraint_acceptance_tolerance | .00002 | First-command rotor-input bound tolerance |
| fallback | geometric defaults | Same documented geometric gains; force-only nominal fallback |

Tune on a separate development task, then freeze before evaluating. Increasing horizon/model complexity changes compute cost; dropping integration resolution can make the motor-lag prediction unstable. Owned config overrides are strict. Solver cache keys include configuration, measured vehicle parameters, implementation source and native-library receipt; builds stay under ignored `.local/acados-generated` (or `IET_CH5_ACADOS_CACHE`). A missing dependency fails clearly instead of substituting another method.

Install and run via [the acados guide](../../../../docs/rigid_body_nmpc.md). [Source manifest](../../../../docs/source_manifest.json) and [third-party notices](../../../../THIRD_PARTY_NOTICES.md) retain pins and licences. The rigid-body OCP is original project code; solver dependencies are unmodified external installations. No new blanket project licence is assigned.
