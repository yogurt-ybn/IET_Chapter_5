# MPC

Near-hover linear constrained MPC using OSQP. Same coupled 12-state model as LQR, full rotor-allocation polytope, speed/tilt component boxes, terminal DARE cost, shifted warm start. 30 x 36 ms preview; solves every 36 ms, holds input between solves. Failed/timed-out QP falls back explicitly to LQR through the shared allocator; fallback need not satisfy state constraints. No nonlinear-MPC, stability or recursive-feasibility claim.

`controller.py` contains the numerical algorithm; `config.json` is its authoritative default. No simulator API or allocator is duplicated here. Call `make_controller(name, config)` then `reset(state, reference, parameters)` and `update(state, reference, dt)`.

| Parameter | Default | Units / meaning |
|---|---|---|
| `design_dt` | `0.036` | s; Discrete model period; redesign gains/constraints when changing. |
| `Q` | `[8, 8, 12, 4, 4, 5, 12, 12, 8, 0.3, 0.3, 0.3]` | inverse squared state units; Diagonal state cost in order [p(3),v(3),roll,pitch,yaw,omega(3)]. |
| `W` | `[0.5, 50, 50, 80]` | inverse squared input units; Diagonal wrench-deviation cost [delta f,tau_x,tau_y,tau_z]. |
| `horizon` | `30` | steps; Prediction duration = horizon * design_dt. |
| `speed_limit` | `2.0` | m/s norm; QP uses component boxes limit/sqrt(3). |
| `tilt_limit_rad` | `0.3` | rad norm approximation; QP small-angle roll/pitch boxes limit/sqrt(2). |
| `eps_abs` | `1e-05` | solver scaled residual; OSQP absolute stopping tolerance. |
| `eps_rel` | `1e-05` | dimensionless; OSQP relative stopping tolerance. |
| `max_iter` | `2000` | iterations; OSQP maximum iterations before explicit LQR fallback. |
| `solver_time_limit_s` | `0.008` | wall s; OSQP solve time cap; timeout activates explicit LQR fallback. |
| `constraint_acceptance_tolerance` | `0.0002` | mixed constraint units; Post-solve max residual tolerance on rotor/state constraints. |

Tune on a separate nominal task, then freeze overrides before evaluation. Keep shared vehicle/allocation/feedback settings identical across controllers. Use the unified CLI with `--controller-overrides configs/experiments/tuning.example.json`; it rejects unknown keys and wrong vector lengths. Do not edit configurations between repeats.

Source and licence: [root notices](../../../../THIRD_PARTY_NOTICES.md), [source manifest](../../../../docs/source_manifest.json), [equation mapping](../../../../docs/controller_mapping.md). The numerical law is preserved from the pre-refactor implementation; only package imports/configuration ownership changed. Original project code has no new blanket licence.
