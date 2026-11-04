# LQR

Coupled 12-state discrete near-hover model, exact ZOH discretization and offline DARE. Actual-minus-reference state deviation with negative feedback; hover input mg. Uses p/v/a/jerk and heading/rate. Large angles exceed the model assumptions.

`controller.py` contains the numerical algorithm; `config.json` is its authoritative default. No simulator API or allocator is duplicated here. Call `make_controller(name, config)` then `reset(state, reference, parameters)` and `update(state, reference, dt)`.

| Parameter | Default | Units / meaning |
|---|---|---|
| `design_dt` | `0.012` | s; Discrete model period; redesign gains/constraints when changing. |
| `Q` | `[8, 8, 12, 4, 4, 5, 12, 12, 8, 0.3, 0.3, 0.3]` | inverse squared state units; Diagonal state cost in order [p(3),v(3),roll,pitch,yaw,omega(3)]. |
| `W` | `[0.5, 50, 50, 80]` | inverse squared input units; Diagonal wrench-deviation cost [delta f,tau_x,tau_y,tau_z]. |

Tune on a separate nominal task, then freeze overrides before evaluation. Keep shared vehicle/allocation/feedback settings identical across controllers. Use the unified CLI with `--controller-overrides configs/experiments/tuning.example.json`; it rejects unknown keys and wrong vector lengths. Do not edit configurations between repeats.

Source and licence: [root notices](../../../../THIRD_PARTY_NOTICES.md), [source manifest](../../../../docs/source_manifest.json), [equation mapping](../../../../docs/controller_mapping.md). The numerical law is preserved from the pre-refactor implementation; only package imports/configuration ownership changed. Original project code has no new blanket licence.
