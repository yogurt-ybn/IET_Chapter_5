# Modular UAV case study

Measured AirSim runs: 2. Status counts: `{'passed': 2}`.

Source commit: `692f6e9fde2ad87ec6006134c711d5299ed99a41`; executable/config content SHA256: `46b6bfcd0778a8656c6e369d92b859b608bc5773c281a884c1f8e2114cc2604b`.

All controllers use the same verified motor allocation and independent physics evaluation. Paused simulation timing does not establish real-time feasibility. Controller-owned defaults are frozen in protocol.json. Aggressive flight is evaluated separately from the near-hover design envelope; no claim of statistical superiority.

| Trajectory | Controller | Pass / runs | Tracking RMS range (m), complete runs |
|---|---|---:|---:|
| transfer (nominal) | nmpc | 1 / 1 | 0.0001–0.0001 |
| agile_course (aggressive) | nmpc | 1 / 1 | 0.0038–0.0038 |

Repeatability checks: 0 / 0 within the predeclared position-delta tolerance. Two repeats are a reproducibility check, not a powered statistical comparison.

Aborted runs retain their available-interval metrics and failure reason, but are excluded from complete-interval RMS comparisons. All failed complete runs remain in tables/plots. Raw control/physics logs remain in the ignored local archive; raw_log_inventory.json records their individual hashes. External storage upload is not claimed.

Figures can be regenerated from the committed NPZ files with `scripts/plot_case_study.py`. Force-pulse injection remains unavailable; wind, mismatch and paired noise/delay protocols are configured separately.
