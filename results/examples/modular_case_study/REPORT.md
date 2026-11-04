# Modular UAV case study

Measured AirSim runs: 70. Status counts: `{'passed': 64, 'tracking_failed': 6}`.

Source commit: `25f9c7a6926d746c3742e60233772f6b486bfd5d`; executable/config content SHA256: `6aaf79258ebb8834cd195810be33db9fe9fe966b6ac4750166a85aed716a73ce`.

All controllers use the same verified motor allocation and independent physics evaluation. Paused simulation timing does not establish real-time feasibility. Controller-owned defaults are frozen in protocol.json. Aggressive flight is evaluated separately from the near-hover design envelope; no claim of statistical superiority.

| Trajectory | Controller | Pass / runs | Tracking RMS range (m), complete runs |
|---|---|---:|---:|
| hover (nominal) | pid | 2 / 2 | 0.0000–0.0000 |
| hover (nominal) | lqr | 2 / 2 | 0.0000–0.0000 |
| hover (nominal) | geometric | 2 / 2 | 0.0000–0.0000 |
| hover (nominal) | dob | 2 / 2 | 0.0000–0.0000 |
| hover (nominal) | mpc | 2 / 2 | 0.0000–0.0000 |
| transfer (nominal) | pid | 2 / 2 | 0.0119–0.0119 |
| transfer (nominal) | lqr | 2 / 2 | 0.0010–0.0010 |
| transfer (nominal) | geometric | 2 / 2 | 0.0004–0.0004 |
| transfer (nominal) | dob | 2 / 2 | 0.0003–0.0003 |
| transfer (nominal) | mpc | 2 / 2 | 0.0038–0.0038 |
| circle (nominal) | pid | 2 / 2 | 0.0600–0.0600 |
| circle (nominal) | lqr | 2 / 2 | 0.0097–0.0097 |
| circle (nominal) | geometric | 2 / 2 | 0.0010–0.0010 |
| circle (nominal) | dob | 2 / 2 | 0.0006–0.0006 |
| circle (nominal) | mpc | 2 / 2 | 0.0101–0.0101 |
| figure_eight (nominal) | pid | 2 / 2 | 0.0817–0.0817 |
| figure_eight (nominal) | lqr | 2 / 2 | 0.0157–0.0157 |
| figure_eight (nominal) | geometric | 2 / 2 | 0.0015–0.0015 |
| figure_eight (nominal) | dob | 2 / 2 | 0.0008–0.0008 |
| figure_eight (nominal) | mpc | 2 / 2 | 0.0165–0.0165 |
| helix (nominal) | pid | 2 / 2 | 0.0306–0.0306 |
| helix (nominal) | lqr | 2 / 2 | 0.0035–0.0035 |
| helix (nominal) | geometric | 2 / 2 | 0.0007–0.0007 |
| helix (nominal) | dob | 2 / 2 | 0.0004–0.0004 |
| helix (nominal) | mpc | 2 / 2 | 0.0047–0.0047 |
| spatial_course (nominal) | pid | 2 / 2 | 0.0622–0.0622 |
| spatial_course (nominal) | lqr | 2 / 2 | 0.0387–0.0387 |
| spatial_course (nominal) | geometric | 2 / 2 | 0.0020–0.0020 |
| spatial_course (nominal) | dob | 2 / 2 | 0.0017–0.0017 |
| spatial_course (nominal) | mpc | 2 / 2 | 0.0436–0.0436 |
| agile_course (aggressive) | pid | 0 / 2 | 1.1159–1.1159 |
| agile_course (aggressive) | lqr | 0 / 2 | 0.3150–0.3150 |
| agile_course (aggressive) | geometric | 2 / 2 | 0.0153–0.0153 |
| agile_course (aggressive) | dob | 2 / 2 | 0.0078–0.0078 |
| agile_course (aggressive) | mpc | 0 / 2 | 0.3372–0.3376 |

Repeatability checks: 34 / 35 within the predeclared position-delta tolerance. Two repeats are a reproducibility check, not a powered statistical comparison.

Aborted runs retain their available-interval metrics and failure reason, but are excluded from complete-interval RMS comparisons. All failed complete runs remain in tables/plots. Raw control/physics logs remain in the ignored local archive; raw_log_inventory.json records their individual hashes. External storage upload is not claimed.

Figures can be regenerated from the committed NPZ files with `scripts/plot_case_study.py`. Force-pulse injection remains unavailable; wind, mismatch and paired noise/delay protocols are configured separately.
