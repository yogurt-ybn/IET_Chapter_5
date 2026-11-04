# L0: existing NMPC baseline verified

Date: 4 November 2026. Numerical source: `692f6e9fde2ad87ec6006134c711d5299ed99a41`.

A newly launched, dedicated AirSim Blocks instance was capability-checked before flight. Existing AirSim source, binary and local-diff hashes match the previous preservation inventory. No pre-existing simulator process was found or commanded.

| Trajectory | Traversal RMS (m) | Complete | Fallbacks | Status |
|---|---:|---|---:|---|
| transfer | 0.000123272 | True | 0 | passed |
| agile_course | 0.003836491 | True | 0 | passed |

These flights use the existing 17-state rigid-body NMPC, default 40-stage / 24 ms prediction, common 12 ms feedback and physical motor adapter. No learning model was fitted or used. Genuine raw control/physics logs remain local; compact independent truth/configuration/metrics are in `results/examples/learning_baseline/`. Paused stepping is not a real-time validation.

An additional 10-flight acquisition uses transfer, circle and helix for training, and separate figure-eight and spatial-course flights for independent prediction validation, in nominal and 10% controller-mass-bias conditions. The physical AirSim vehicle is unchanged; inertia bias is disabled. All 10 complete flights passed tracking acceptance. They are acquisition evidence, not learned-controller results.
