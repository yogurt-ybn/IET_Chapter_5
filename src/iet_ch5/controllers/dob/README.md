# DOB

Bounded force-only disturbance observer around the exact geometric nominal controller. Implicit-Euler momentum-increment residual uses interval-mean telemetry-estimated applied collective. Estimate is subtracted from the inertial force before attitude construction; no moment observer or estimator validation is claimed.

`controller.py` contains the numerical algorithm; `config.json` is its authoritative default. No simulator API or allocator is duplicated here. Call `make_controller(name, config)` then `reset(state, reference, parameters)` and `update(state, reference, dt)`.

| Parameter | Default | Units / meaning |
|---|---|---|
| `enabled` | `true` | boolean; False disables force compensation with identical nominal gains. |
| `bandwidth_rad_s` | `5.0` | rad/s; Force residual filter bandwidth; higher values amplify noise/lag. |
| `force_bound_N` | `2.0` | N per axis; Clips the NWU force estimate; no torque estimation. |
| `nominal.position_kp` | `[4.0, 4.0, 5.0]` | N/m; NWU x/y/z position error to inertial force gains; increase only after velocity/attitude loops are stable. |
| `nominal.velocity_kp` | `[3.0, 3.0, 3.5]` | N s/m; NWU velocity error to inertial force gain. |
| `nominal.attitude_kp` | `[0.6, 0.7, 0.35]` | N m/rad; FLU attitude gains; PID produces rate target, geometric produces moment. |
| `nominal.rate_kp` | `[0.12, 0.14, 0.1]` | N m s/rad; FLU rate feedback; PID scales angular acceleration by inertia. |

Tune on a separate nominal task, then freeze overrides before evaluation. Keep shared vehicle/allocation/feedback settings identical across controllers. Use the unified CLI with `--controller-overrides configs/experiments/tuning.example.json`; it rejects unknown keys and wrong vector lengths. Do not edit configurations between repeats.

Source and licence: [root notices](../../../../THIRD_PARTY_NOTICES.md), [source manifest](../../../../docs/source_manifest.json), [equation mapping](../../../../docs/controller_mapping.md). The numerical law is preserved from the pre-refactor implementation; only package imports/configuration ownership changed. Original project code has no new blanket licence.
