# PID

Cascaded position/velocity and attitude/rate feedback with bounded integrals and allocation-aware anti-windup. Uses p/v/a and heading/rate. Outer loop 36 ms; inner loop measured 12 ms. Acceleration feedforward is component-clipped.

`controller.py` contains the numerical algorithm; `config.json` is its authoritative default. No simulator API or allocator is duplicated here. Call `make_controller(name, config)` then `reset(state, reference, parameters)` and `update(state, reference, dt)`.

| Parameter | Default | Units / meaning |
|---|---|---|
| `outer_period` | `0.036` | s; Position/velocity-loop period; rate loop uses measured simulation dt. |
| `position_kp` | `[0.9, 0.9, 1.1]` | s^-1; NWU x/y/z position gains; increase only after velocity/attitude loops are stable. |
| `velocity_kp` | `[2.5, 2.5, 3.0]` | s^-1; NWU velocity error to acceleration gain. |
| `velocity_ki` | `[0.12, 0.12, 0.2]` | s^-2; Velocity integral gain; watch actuator/acceleration clipping. |
| `velocity_limit` | `2.0` | m/s per axis; PID target-velocity component clipping. |
| `velocity_integral_limit` | `1.0` | m; Bound on integrated velocity error per component. |
| `acceleration_limit` | `2.5` | m/s^2 per axis; PID requested acceleration component limit. |
| `attitude_kp` | `[7.0, 7.0, 5.0]` | s^-1; FLU attitude gains; PID produces rate target, geometric produces moment. |
| `rate_kp` | `[18.0, 18.0, 12.0]` | s^-1; FLU rate feedback; PID scales angular acceleration by inertia. |
| `rate_ki` | `[0.3, 0.3, 0.3]` | s^-2; FLU rate integral gain before inertia scaling. |
| `rate_limit` | `2.0` | rad/s per axis; PID body-rate target clipping. |
| `rate_integral_limit` | `0.5` | rad; Bound on integrated body-rate error per component. |
| `rate_filter_tc` | `0.008` | s; First-order measured rate low-pass time constant. |

Tune on a separate nominal task, then freeze overrides before evaluation. Keep shared vehicle/allocation/feedback settings identical across controllers. Use the unified CLI with `--controller-overrides configs/experiments/tuning.example.json`; it rejects unknown keys and wrong vector lengths. Do not edit configurations between repeats.

Source and licence: [root notices](../../../../THIRD_PARTY_NOTICES.md), [source manifest](../../../../docs/source_manifest.json), [equation mapping](../../../../docs/controller_mapping.md). The numerical law is preserved from the pre-refactor implementation; only package imports/configuration ownership changed. Original project code has no new blanket licence.
