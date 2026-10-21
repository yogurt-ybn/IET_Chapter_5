# Modular UAV case study

Measured AirSim runs: 42. Status counts: `{'passed': 40, 'tracking_failed': 2}`.

Source commit: `99515454ff0c69bce66f4a8f9ca6af5989057d25`; executable/config content SHA256: `3fdc16453f7fac5c3dc0c5895e229278afc960b3372c22cae315b08d6436bbfe`.

All controllers use the same verified motor allocation and independent physics evaluation. Paused simulation timing does not establish real-time feasibility. Controller-owned defaults are frozen in protocol.json. Aggressive flight is evaluated separately from the near-hover design envelope; no claim of statistical superiority.

| Trajectory | Controller | Pass / runs | Tracking RMS range (m), complete runs |
|---|---|---:|---:|
| hover (nominal) | mpc | 2 / 2 | 0.0000–0.0000 |
| hover (nominal) | nmpc | 2 / 2 | 0.0000–0.0000 |
| hover (nominal) | geometric | 2 / 2 | 0.0000–0.0000 |
| transfer (nominal) | mpc | 2 / 2 | 0.0038–0.0038 |
| transfer (nominal) | nmpc | 2 / 2 | 0.0001–0.0001 |
| transfer (nominal) | geometric | 2 / 2 | 0.0004–0.0004 |
| circle (nominal) | mpc | 2 / 2 | 0.0101–0.0101 |
| circle (nominal) | nmpc | 2 / 2 | 0.0003–0.0003 |
| circle (nominal) | geometric | 2 / 2 | 0.0010–0.0010 |
| figure_eight (nominal) | mpc | 2 / 2 | 0.0165–0.0165 |
| figure_eight (nominal) | nmpc | 2 / 2 | 0.0005–0.0005 |
| figure_eight (nominal) | geometric | 2 / 2 | 0.0015–0.0015 |
| helix (nominal) | mpc | 2 / 2 | 0.0047–0.0047 |
| helix (nominal) | nmpc | 2 / 2 | 0.0002–0.0002 |
| helix (nominal) | geometric | 2 / 2 | 0.0007–0.0007 |
| spatial_course (nominal) | mpc | 2 / 2 | 0.0436–0.0436 |
| spatial_course (nominal) | nmpc | 2 / 2 | 0.0004–0.0004 |
| spatial_course (nominal) | geometric | 2 / 2 | 0.0020–0.0020 |
| agile_course (aggressive) | mpc | 0 / 2 | 0.3318–0.3321 |
| agile_course (aggressive) | nmpc | 2 / 2 | 0.0038–0.0038 |
| agile_course (aggressive) | geometric | 2 / 2 | 0.0153–0.0153 |

Repeatability checks: 20 / 21 within the predeclared position-delta tolerance. Two repeats are a reproducibility check, not a powered statistical comparison.

Aborted runs retain their available-interval metrics and failure reason, but are excluded from complete-interval RMS comparisons. All failed complete runs remain in tables/plots. Raw control/physics logs remain in the ignored local archive; raw_log_inventory.json records their individual hashes. External storage upload is not claimed.

Figures can be regenerated from the committed NPZ files with `scripts/plot_case_study.py`. Force-pulse injection remains unavailable; wind, mismatch and paired noise/delay protocols are configured separately.
