# Faster figure-eight tracking with sustained cruise

The same 4×2 m figure eight now has a 14 s reference traversal instead of 28 s. The old
whole-manoeuvre smoothstep is replaced by 2 s of smooth phase-rate acceleration, 10 s at
constant phase rate, and 2 s of smooth deceleration. Including unchanged 2 s initial/final
holds, the simulator run is 18 s instead of 32 s. The shape, fixed local altitude/heading,
controller gains, plant, truth-feedback pipeline and motor allocation are unchanged.
The [original slower experiment](figure_eight_demo.md) remains available.

## Time law and speeds

The selected configuration is `configs/figure-eight-cruise-task.json`. Let `tau = t - 2`,
`T = 14`, `r = 2`, and `omega_c = 2*pi/(T-r) = pi/6` rad/s. With
`S(u) = 126u^5 - 420u^6 + 540u^7 - 315u^8 + 70u^9` and `I(u) = integral_0^u S(q)dq`,

```text
theta = 0                                      tau <= 0
theta = omega_c*r*I(tau/r)                      0 < tau < r
theta = omega_c*(tau-r/2)                       r <= tau <= T-r
theta = 2*pi-omega_c*r*I((T-tau)/r)             T-r < tau < T
theta = 2*pi                                   tau >= T
p_d(t) = [2*(cos(theta)-1), sin(2*theta), 2] m   local NWU
```

Since `I(1)=1/2`, the ramps and cruise integrate to exactly one turn. Reference position,
velocity, acceleration, jerk and snap are analytic and continuous across both joins and
at rest. Global simulation times are: hold 0–2 s, accelerate 2–4 s, cruise 4–14 s,
decelerate 14–16 s, hold 16–18 s. Centre crossings occur at 6 and 12 s. Constant phase
rate does not mean constant Cartesian speed on a figure eight; the middle retains the
geometric variation in speed while avoiding the earlier long slow entry and exit.

The common path length is approximately 12.194 m. Reference mean speed during traversal
increases from 0.436 to 0.871 m/s; peak speed is 1.481 m/s and peak acceleration about
1.264 m/s². Peak absolute velocity components are each 1.047 m/s, below the unchanged
MPC conservative component bound `2/sqrt(3)` m/s. No velocity/tilt limits or controller
gains are relaxed to obtain this timing. This remains a near-hover model comparison,
rather than an aggressive-flight or hardware-flight test.

## Actual evidence

Each controller follows the new reference through the verified AirSim motor bridge.
Fixed-camera scene capture still shows yellow reference and cyan flown trace, and each
panel now labels its actual instantaneous physics speed in m/s together with time/error.
The video header distinguishes acceleration, cruise and deceleration. The MP4 plays at
1× simulation time, while the compact GIF preview is explicitly 2×. The change in actual
simulation-time reference duration is independent of that preview playback setting.

The speed comparison uses independent physics velocities on the common 12 ms grid.
Reported mean speed is the time integral of velocity norm divided by the traversal
interval: 2–30 s for the old reference and 2–16 s for the new reference. Both old and new
speed statistics are processed with the same implementation. Published compact samples
retain truth/reference velocities so the speed curves can be audited without confusing
commanded speed with measured flight speed. All images must have exact paused-physics
timestamps, matching source hashes and the same fixed camera pose.

The first capture client stopped with shell exit code 143 during LQR's final hold;
the simulator remained running. SIGTERM is indicated by that exit code, but its sender
and cause are unverified. The partial LQR record is retained separately and excluded
from the complete comparison. Recovery disarmed/paused only the selected dedicated
simulator, kept the completed PID record, and restarted LQR and the remaining controllers
from the common initialization with unchanged source/configuration. The client interruption
and recovery driver are retained with the result provenance.

## Reproduce

Launch a dedicated simulator and wait for the scene to load before checking capabilities:

```bash
.venv/bin/python scripts/launch_simulator.py --local-config .local/environment.json --settings configs/airsim-demo-settings.json --local-output .local/figure-eight-cruise-simulator
.venv/bin/python scripts/check_environment.py --local-config .local/environment.json --output .local/figure-eight-cruise-capabilities.json
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/capture_tracking_comparison.py --local-config .local/environment.json --exclusive-simulator --task-config configs/figure-eight-cruise-task.json --output results/runs/my-cruise-eight
.venv/bin/python scripts/render_tracking_comparison.py --source results/runs/my-cruise-eight --output results/runs/my-cruise-eight-publication
.venv/bin/python scripts/plot_speed_comparison.py --previous results/runs/figure-eight-feasible-20261020 --current results/runs/my-cruise-eight-publication
```

Output directories must be new. The last command needs the retained old raw records
(or those extracted from its verified local archive). It stores reprocessed old speed
samples beside the new publication without rewriting the original published experiment.
Capturing/benchmarking uses paused simulator time and makes no real-time claim. Raw scene
images and full logs remain in ignored local archives; compact data/media are published.

## Measured delivery

[18 s normal-speed video](../results/examples/figure_eight_cruise/comparison.mp4), [2× GIF preview](../results/examples/figure_eight_cruise/comparison.gif), [speed comparison](../results/examples/figure_eight_cruise/speed_comparison.png), [trajectory/error figure](../results/examples/figure_eight_cruise/trajectories_and_errors.png) and [machine-readable speed statistics](../results/examples/figure_eight_cruise/speed_comparison.json).

| Controller | Previous mean (m/s) | New mean (m/s) | New peak (m/s) | Mean-speed ratio |
| --- | ---: | ---: | ---: | ---: |
| PID | 0.4434 | 0.9126 | 1.5317 | 2.058 |
| LQR | 0.4353 | 0.8706 | 1.4792 | 2.000 |
| Geometric SE(3) | 0.4354 | 0.8706 | 1.4793 | 2.000 |
| Geometric + force DOB | 0.4355 | 0.8709 | 1.4802 | 2.000 |
| Constrained linear MPC | 0.4353 | 0.8707 | 1.4791 | 2.000 |

The recorded velocities confirm approximately double the actual traversal mean speed. PID's larger measured speed accompanies larger tracking excursions; it is not an accuracy improvement. All five complete 18 s and pass the fixed 0.1 m traversal RMS threshold. There are no collisions, motor saturation, evaluated speed/tilt violations or MPC fallback activations; all 500 MPC QP updates report solved. The PID peak error exceeds 0.1 m, which is retained; acceptance is an RMS threshold, not a maximum-error threshold.

| Controller | Whole-run position RMS (mm) | Maximum position error (mm) |
| --- | ---: | ---: |
| PID | 46.234 | 109.143 |
| LQR | 7.437 | 11.935 |
| Geometric SE(3) | 1.433 | 2.855 |
| Geometric + force DOB | 0.664 | 1.350 |
| Constrained linear MPC | 7.831 | 12.587 |

All five complete runs use identical clean source `fe88593`. Independent common-grid samples include truth and reference velocities. The 905 actual native images have exact paused-physics timestamps and matching camera poses. The MP4 fully decodes to 180 frames at 1600×1008, 10 fps, 18 s; the GIF to 90 frames over 9 s at 2×. Maximum selected-scene age is 0.1 s. The 28-test suite including read-only AirSim integration passes; lint and whitespace checks pass.

Full raw records, including the interrupted LQR attempt, are retained in the local archive identified by [size and SHA256](../results/examples/figure_eight_cruise/full_archive.json). The [client-interruption record](../results/examples/figure_eight_cruise/client_interruption.json) and [exact recovery driver/provenance](../results/examples/figure_eight_cruise/capture_recovery_provenance.json) are public. No external upload of the large archive is claimed. The original 51-run matrix and slower demonstrations are unchanged.
