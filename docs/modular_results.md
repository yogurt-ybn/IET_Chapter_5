# Measured modular case-study results

Numerical/configuration freeze: `25f9c7a6926d746c3742e60233772f6b486bfd5d` with the content fingerprint in [protocol.json](../results/examples/modular_case_study/protocol.json). Per-run metadata retains actual source-state records; the final postprocessor additionally improves figure text contrast and the video reference inset; no controller, reference or runtime configuration was changed. `analysis_provenance.json` identifies the postprocessor used. The original 51-run M3 dataset remains separate and unchanged.

## Outcome

All 70 actual AirSim runs completed their common intervals: **64 tracking passes and six tracking-threshold failures**. There were no bridge failures, collision/safety aborts, rotor saturation or violations of the declared task-level speed/tilt evaluation limits. The 60 nominal runs (six trajectories × five methods × two repeats) all passed traversal RMS <0.10 m. Results use independent physics truth on the same 12 ms evaluation grid, including the exact endpoint.

The aggressive task is a 6 m wide 3D multi-harmonic figure eight with varying heading, 8.5 s traversal and 1 s initial/final holds. Its reference peak speed is approximately 4.02 m/s. Physical feedforward screening passed, while the PID limits and near-hover linear models/constraints remained unchanged. The aggressive threshold was frozen at traversal RMS <0.15 m.

| Method | Aggressive traversal RMS, repeats (m) | Largest position error (m) | Passes |
|---|---:|---:|---:|
| PID | 1.11594 / 1.11594 | 2.16937 | 0/2 |
| LQR | 0.31503 / 0.31503 | 0.52652 | 0/2 |
| Geometric SE(3) | 0.01530 / 0.01530 | 0.02498 | 2/2 |
| Geometric + force DOB | 0.00778 / 0.00778 | 0.01211 | 2/2 |
| Constrained linear MPC with explicit LQR fallback | 0.33724 / 0.33763 | 0.50969 | 0/2 |

These are outcomes for this vehicle, configuration and selected trajectories, not statistical claims about controller families. PID's component acceleration/velocity/rate clipping and the linear controllers' near-hover assumptions remain relevant limitations. No controller was retuned after observing the evaluation results. The physically infeasible initial reference candidate was rejected before the numerical freeze; its screen and separate pilot are retained in [preflight evidence](../results/examples/modular_preflight/README.md).

## MPC and repeatability limits

Every nominal MPC QP update solved without fallback. In the aggressive task, only 84/86 of 292 updates per run were accepted as solved; **208/206 updates used explicit LQR fallback**. The first run reported 188 primal-infeasible, 2 primal-infeasible-inaccurate, 7 time-limit and 11 solved-inaccurate statuses. The second reported 187, 2, 3 and 14 respectively. Solved-inaccurate is rejected by the controller's frozen acceptance policy. Thus aggressive MPC traces are results of the documented MPC/fallback policy, not evidence that the constrained optimizer tracked the entire trajectory successfully. The same allocator bounds every requested wrench, including fallback output.

**34/35 paired repeatability checks passed** the predeclared maximum position-delta limit of 0.02 m. All 30 nominal pairs passed. The aggressive MPC pair failed with 0.038045 m maximum delta; its two RMS values also differ. Wall-time solver caps and resulting fallback decisions are not deterministic; status counts differ here. The records do not establish aggressive MPC repeatability. The other pairs differ by at most approximately 0.0000051 m. Failed checks remain in [repeatability.json](../results/examples/modular_case_study/repeatability.json).

Controller computation p95 across nominal runs ranged from 0.178–0.264 ms (PID), 0.224–0.349 ms (LQR), 0.379–0.586 ms (geometric), 0.402–0.656 ms (DOB), and 1.104–4.128 ms (MPC). These are paused-benchmark computation measurements. Total wall-loop deadlines and RPC/physics overhead are recorded separately; no real-time-flight claim follows from controller computation time alone.

## Inspect and reproduce

- [Complete tables and compact run records](../results/examples/modular_case_study/REPORT.md)
- [Accuracy matrix](../results/examples/modular_case_study/figures/accuracy_matrix.png)
- [Aggressive 3D / top-down trajectories and errors](../results/examples/modular_case_study/figures/agile_course_tracking.png)
- [Requested/estimated wrench, motor range and speed](../results/examples/modular_case_study/figures/agile_course_inputs.png)
- [Separate actual-scene comparison video](../results/examples/modular_agile_demo/comparison.mp4), [2x GIF](../results/examples/modular_agile_demo/comparison.gif)
- [Runnable architecture, tuning and commands](modular_case_study.md)

The five scene-capture runs are independent of the 70 timing/evaluation runs and have their own frozen manifest and measured outcomes. They do not increase benchmark repeat counts. Every source image hash, fixed camera pose and paused image/truth timestamp is checked before rendering; full MP4 decoding is checked afterward. The native yellow path is the preset reference and cyan is the actual trace. If a run aborts, the renderer holds its last real frame and explicitly labels it stopped; no fabricated continuation is drawn.

PDF/SVG/PNG figures regenerate from committed NPZ samples using `scripts/plot_case_study.py`. Full raw control/physics/frame records are kept in the checksummed ignored local archive identified by `full_archive.json`; no external archive upload is claimed. Wind, mismatch and five paired noise/delay protocols are available separately. Their earlier measured evidence remains in the original M3 case study; they were not rerun as part of this new 70-run nominal/aggressive matrix. Prescribed external force-pulse injection and physical-flight transfer remain unvalidated.
