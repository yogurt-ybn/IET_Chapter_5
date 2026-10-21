# Measured rigid-body NMPC and acados results

Numerical/runtime/configuration freeze: `99515454ff0c69bce66f4a8f9ca6af5989057d25`. The original 70-run and 51-run reports remain unchanged. New postprocessing records its own source separately in analysis_provenance.json.

## Closed-loop AirSim evaluation

The new matrix contains **42 genuine AirSim runs: 40 tracking passes and two tracking-threshold failures**. All intervals completed. The two failures are the historical near-hover linear MPC on the aggressive task. All 14 rigid-body NMPC runs pass, with zero geometric fallback. All 36 nominal runs pass. There is no collision/safety/bridge abort, actuator saturation or declared task-level speed/tilt evaluation violation. Independent physics truth is resampled on the common 12 ms grid including endpoints.

| Trajectory | Linear MPC RMS (mm), two repeats | Rigid-body NMPC RMS (mm), two repeats | Geometric RMS (mm), two repeats |
|---|---:|---:|---:|
| hover | 0.000 / 0.000 | 0.000 / 0.000 | 0.000 / 0.000 |
| transfer | 3.824 / 3.824 | 0.072 / 0.072 | 0.382 / 0.382 |
| circle | 10.090 / 10.089 | 0.341 / 0.342 | 1.005 / 1.005 |
| figure_eight | 16.458 / 16.460 | 0.475 / 0.475 | 1.472 / 1.472 |
| helix | 4.688 / 4.687 | 0.202 / 0.203 | 0.699 / 0.699 |
| spatial_course | 43.614 / 43.615 | 0.430 / 0.431 | 1.955 / 1.956 |
| agile_course | 331.756 / 332.115 | 3.837 / 3.840 | 15.297 / 15.297 |

The aggressive reference peaks at 4.02 m/s with roughly 40 degrees of ideal thrust tilt. Its 8.5 s 3D multi-harmonic course, heading profile and 0.15 m acceptance limit are unchanged. NMPC achieves **3.837 / 3.840 mm traversal RMS**, **8.095 / 8.105 mm maximum position error**, and zero fallback. The linear MPC policy remains near 0.332 m RMS and uses 231 LQR fallback activations per run. The geometric baseline remains approximately 15.3 mm RMS.

**20/21 paired repeatability checks pass** the declared 20 mm maximum trajectory-delta limit. All seven NMPC pairs pass; its largest delta is about 0.0215 mm on the aggressive task. The aggressive linear MPC pair fails at 34.379 mm. Differing wall-time solver/fallback decisions remain auditable, not silently retuned.

This is an implementation comparison: the nonlinear model, actuator-lag prediction, cost weights, update interval (24 ms versus 36 ms), preview horizon and state-constraint policy differ from the old linear baseline. Both share the actual plant, initial state, reference, feedback information, actuator adapter and experiment limits. It does not isolate nonlinear modeling alone or establish a general ranking of controller families. Default NMPC weights were not retuned from final evaluation results; separate pre-freeze development evidence is retained.

## Computation and acceleration

On the aggressive closed-loop runs, native C-solver p95 is **3.632 / 3.798 ms**. Complete optimization-update computation p95 is **10.031 / 11.733 ms**, with maxima **91.726 / 85.491 ms**. These include Python stage/parameter updates, solution extraction and diagnostics. Preparation/code generation is outside evaluated control and recorded separately. The host is not CPU-isolated; occasional wall-time spikes and paused simulator stepping mean **hard real-time operation is not established**.

A separate offline replay uses **438 measured feedback/input records x three repeats per variant**, with identical model, horizon, physical input constraints and cost. Each variant performs 1,314 calls; all return accepted status with zero fallback. The work is replayed optimizer computation, not additional closed-loop flight. Hardware/thread settings and exact compressed feedback are committed.

| Variant | Native solver p95 (ms) | Complete call p95 (ms) |
|---|---:|---:|
| rti_warm_condensed | 2.462 | 7.285 |
| sqp_warm_condensed | 4.373 | 8.420 |
| rti_cold_condensed | 2.233 | 6.737 |
| rti_warm_uncondensed | 2.212 | 6.424 |

The default RTI native p95 is about **1.78x faster** than the full-SQP variant; complete-call p95 improves by about **13.5%** on this workload. SQP needs at most two iterations here, so the fully converged warm-started baseline is already relatively inexpensive.

RTI is a single SQP step, not an equally converged NLP solution: its stationarity residual p95 is approximately 0.00203 versus 0.0000930 for SQP. The mean normalized motor-input-vector difference from SQP is about 0.000530, with maximum about 0.00909. Read speed together with these residuals, command differences and measured closed-loop accuracy.

**Warm start and partial condensing do not automatically improve wall time.** In this measured near-reference replay, cold RTI and uncondensed warm RTI are slightly faster than the frozen default. No post-hoc timing choice is substituted into the published closed-loop configuration. The ablation is evidence about tuning tradeoffs, not proof that every acceleration feature helps every problem. Generated C, RTI, HPIPM and BLASFEO are used; asynchronous AS-RTI/Cython-wrapper acceleration is not implemented.

## Artefacts and reproducibility

- [Full 42-run tables](../results/examples/rigid_body_mpc/REPORT.md), [accuracy matrix](../results/examples/rigid_body_mpc/figures/accuracy_matrix.png), [aggressive tracking](../results/examples/rigid_body_mpc/figures/agile_course_tracking.png).
- [Six-controller actual AirSim video](../results/examples/rigid_body_demo/comparison.mp4), [2x GIF](../results/examples/rigid_body_demo/comparison.gif): six independent capture runs, three passes / three retained failures, separate from matrix timing/repeat counts.
- [Acceleration results](../results/examples/acados_acceleration/REPORT.md), [portable measured-feedback replay](../results/examples/acados_acceleration/REPLAY.md), [timing figure](../results/examples/acados_acceleration/acceleration.png).
- [Model, parameter guide, installation and tested commands](rigid_body_nmpc.md), [development evidence](../results/examples/nmpc_development/README.md).

The video verifies all 528 original image hashes, one fixed camera and exact image/paused-truth timestamps, and fully decodes 106 MP4 frames. Its final postprocessor labels logged QP/RTI and fallback commands; source images are unchanged. Full raw logs/images remain in the local checksummed archive identified by full_archive.json, while compact physics/configuration/results, plots and replay inputs are public. No external raw upload is claimed.

acados v0.5.4 and exact submodules are pinned; installation/compiler/library and generated-solver hashes are retained. The model omits drag/wind prediction, flexible dynamics and time-varying density; nominal motor lag is an approximation to the simulator filter. The finite-horizon terminal cost and status 0 do not prove stability, global optimality or recursive feasibility. New wind/mismatch/noise evaluation, unpaused real-time validation and physical flight are not claimed.
