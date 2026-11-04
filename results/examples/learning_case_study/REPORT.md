# Matched GP force-residual NMPC: measured AirSim results

Numerical freeze: `8e9b2161496b93f7e09c28e6aa8032563750df65`; source/config SHA256 `3087ad01437947f488fcafdeb528ef41f522d0323028354e56474771426eebd6`.

Same rigid-body NMPC with a frozen GP force residual disabled/enabled. All pairs have verified equal initial states, physical/model parameters, trajectory, horizon, cost, constraints, feedback and actuator settings. The sole controller-setting difference is `residual.enabled`.

| Condition | Trajectory | Repeat | GP | Status | Traversal RMS (mm) | Peak (mm) | Fallbacks | Violations (s) | Update p95 (ms) |
|---|---|---:|---|---|---:|---:|---:|---:|---:|
| nominal | figure_eight | 0 | off | passed | 0.475 | 0.858 | 0 | 0.0000 | 4.004 |
| nominal | figure_eight | 0 | on | passed | 0.548 | 1.528 | 0 | 0.0000 | 19.112 |
| nominal | figure_eight | 1 | off | passed | 0.475 | 0.853 | 0 | 0.0000 | 3.749 |
| nominal | figure_eight | 1 | on | passed | 0.548 | 1.528 | 0 | 0.0000 | 19.019 |
| nominal | agile_course | 0 | off | passed | 3.836 | 8.095 | 0 | 0.0000 | 3.685 |
| nominal | agile_course | 0 | on | passed | 3.832 | 8.092 | 0 | 0.0000 | 20.140 |
| nominal | agile_course | 1 | off | passed | 3.836 | 8.095 | 0 | 0.0000 | 3.426 |
| nominal | agile_course | 1 | on | passed | 3.833 | 8.091 | 0 | 0.0000 | 21.578 |
| mass_bias | figure_eight | 0 | off | passed | 23.732 | 23.946 | 0 | 0.0000 | 3.339 |
| mass_bias | figure_eight | 0 | on | passed | 0.797 | 1.962 | 0 | 0.0000 | 18.571 |
| mass_bias | figure_eight | 1 | off | passed | 23.732 | 23.945 | 0 | 0.0000 | 4.825 |
| mass_bias | figure_eight | 1 | on | passed | 0.773 | 1.820 | 0 | 0.0000 | 19.615 |
| mass_bias | agile_course | 0 | off | passed | 25.384 | 30.579 | 0 | 0.0000 | 4.963 |
| mass_bias | agile_course | 0 | on | passed | 4.770 | 8.240 | 0 | 0.0000 | 19.259 |
| mass_bias | agile_course | 1 | off | passed | 25.384 | 30.579 | 0 | 0.0000 | 3.501 |
| mass_bias | agile_course | 1 | on | passed | 4.767 | 8.227 | 0 | 0.0000 | 18.884 |

Repeatability: 8/8 within the predeclared 20 mm maximum trajectory-difference limit.

Every attempted closed-loop run and failed outcome is retained. Traversal windows come from the unchanged trajectory presets; full-duration peak and constraint statistics include initial/final holds. Two repeats check reproducibility, not statistical superiority. Paused stepping, computation spikes and native-build preparation preclude a real-time claim. Learned force is a model-discrepancy estimate, not measured wind. Nominal and mass-bias models are trained separately; neither is presented as an online adaptive or universal model. Raw logs remain in the identified local archive; compact truth/configuration/timing arrays are committed.
