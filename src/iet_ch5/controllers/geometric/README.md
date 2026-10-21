# GEOMETRIC

Standard coupled-yaw SE(3) controller in NWU/FLU. Uses p/v/a/jerk/snap and heading derivatives. Force-vector normalization derivatives provide desired attitude/rate/acceleration; singular construction holds the prior valid attitude. Large manoeuvres still depend on actuator authority and motor lag.

`controller.py` contains the numerical algorithm; `config.json` is its authoritative default. No simulator API or allocator is duplicated here. Call `make_controller(name, config)` then `reset(state, reference, parameters)` and `update(state, reference, dt)`.

| Parameter | Default | Units / meaning |
|---|---|---|
| `position_kp` | `[4.0, 4.0, 5.0]` | N/m; NWU x/y/z position error to inertial force gains; increase only after velocity/attitude loops are stable. |
| `velocity_kp` | `[3.0, 3.0, 3.5]` | N s/m; NWU velocity error to inertial force gain. |
| `attitude_kp` | `[0.6, 0.7, 0.35]` | N m/rad; FLU attitude gains; PID produces rate target, geometric produces moment. |
| `rate_kp` | `[0.12, 0.14, 0.1]` | N m s/rad; FLU rate feedback; PID scales angular acceleration by inertia. |

Tune on a separate nominal task, then freeze overrides before evaluation. Keep shared vehicle/allocation/feedback settings identical across controllers. Use the unified CLI with `--controller-overrides configs/experiments/tuning.example.json`; it rejects unknown keys and wrong vector lengths. Do not edit configurations between repeats.

Source and licence: [root notices](../../../../THIRD_PARTY_NOTICES.md), [source manifest](../../../../docs/source_manifest.json), [equation mapping](../../../../docs/controller_mapping.md). The numerical law is preserved from the pre-refactor implementation; only package imports/configuration ownership changed. Original project code has no new blanket licence.
