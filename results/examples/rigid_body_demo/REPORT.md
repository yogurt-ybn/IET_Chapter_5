# Modular UAV case study

Measured AirSim runs: 6. Status counts: `{'passed': 3, 'tracking_failed': 3}`.

Source commit: `99515454ff0c69bce66f4a8f9ca6af5989057d25`; executable/config content SHA256: `3fdc16453f7fac5c3dc0c5895e229278afc960b3372c22cae315b08d6436bbfe`.

All controllers use the same verified motor allocation and independent physics evaluation. Paused simulation timing does not establish real-time feasibility. Controller-owned defaults are frozen in protocol.json. Aggressive flight is evaluated separately from the near-hover design envelope; no claim of statistical superiority.

| Trajectory | Controller | Pass / runs | Tracking RMS range (m), complete runs |
|---|---|---:|---:|
| agile_course (aggressive) | pid | 0 / 1 | 1.1159–1.1159 |
| agile_course (aggressive) | lqr | 0 / 1 | 0.3150–0.3150 |
| agile_course (aggressive) | geometric | 1 / 1 | 0.0153–0.0153 |
| agile_course (aggressive) | dob | 1 / 1 | 0.0078–0.0078 |
| agile_course (aggressive) | mpc | 0 / 1 | 0.3319–0.3319 |
| agile_course (aggressive) | nmpc | 1 / 1 | 0.0038–0.0038 |

Repeatability checks: 0 / 0 within the predeclared position-delta tolerance. Two repeats are a reproducibility check, not a powered statistical comparison.

Aborted runs retain their available-interval metrics and failure reason, but are excluded from complete-interval RMS comparisons. All failed complete runs remain in tables/plots. Raw control/physics logs remain in the ignored local archive; raw_log_inventory.json records their individual hashes. External storage upload is not claimed.

Figures can be regenerated from the committed NPZ files with `scripts/plot_case_study.py`. Force-pulse injection remains unavailable; wind, mismatch and paired noise/delay protocols are configured separately.
