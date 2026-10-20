# Figure-eight trajectory tracking in AirSim

The primary demonstration now uses a horizontal figure eight. Its two lobes, two centre
crossings and changing curvature require successive turns in opposite directions. Five
actual AirSim runs compare cascaded PID, LQR, geometric SE(3), geometric plus force DOB,
and constrained linear MPC on this same predefined reference. The earlier circular
[tracking demonstration](trajectory_demo.md) and frozen 51-run case study remain available.

## Prescribed reference

The configuration is `configs/figure-eight-demo-task.json`. Each run holds for 2 s,
traverses one complete figure eight over 28 s, then holds for 2 s (32 s total). The shape
spans 4 m north-to-south and 2 m west-to-east at local altitude 2 m, fixed heading zero.
The right-hand tip is the common initial position (0, 0, 2) m; its centre is (-2, 0, 2) m.

In vehicle-origin NWU coordinates, the actual reference is

```text
u = clip((t - 2) / 28, 0, 1)
s(u) = 126u^5 - 420u^6 + 540u^7 - 315u^8 + 70u^9
theta(t) = 2*pi*s(u)
p_d(t) = [2*(cos(theta)-1), sin(2*theta), 2] m
```

This is a Gerono figure eight. The two harmonics are evaluated from the same smooth
phase, with analytic velocity, acceleration, jerk and snap. The phase has zero derivatives
through fourth order at entry/exit. There are no waypoint jumps or stop-and-restart
commands at the centre crossings. Different tangent directions at the two centre visits
are checked numerically, so the path is not a retraced single loop.

Dense reference sampling gives peak Euclidean speed approximately 1.309 m/s and peak
acceleration 1.203 m/s². Peak absolute velocity components are approximately 0.941 and
1.104 m/s. The implemented MPC uses the conservative component limit `2/sqrt(3)` m/s
rather than only a 2 m/s norm limit. The final reference fits that box, and its near-hover
feedforward tilt fits the MPC tilt box. These reference checks do not replace recording
actual solver status, state/actuator constraints and failures in the simulator.

The initial 4×2.4 m / 24 s configuration was stopped during a PID preview after inspecting
the more restrictive MPC component bounds. Its partial records remain locally under
`results/runs/figure-eight-comparison-20261020`; they are excluded from the final comparison.
The selected 4×2 m / 28 s reference was frozen before all five final recordings. The
controller gains, allocation, measured plant and truth-feedback pipeline are unchanged.
No external wind, parameter mismatch or synthetic feedback noise is added in this nominal
shape-tracking test. DOB robustness should still be assessed with its matched wind ablation.

## Scene and synchronization

All five runs start from the same independently verified state after reset/rotor spinup.
All flight motion comes from the shared motor-level wrench adapter. Reference positions
are not sent to an AirSim high-level path-following controller.

A fixed external camera looks down on the complete figure eight. Yellow AirSim markers
show the prescribed path, cyan markers the actual path already flown. Camera position
and orientation are checked at each capture. Vehicle-origin NWU trajectories are rotated
and translated into global NED using the measured actor/vehicle origin offset; the
configured spawn Z alone is insufficient. The camera is at local NED (-2, -3.5, -8) m,
looking toward the path centre, with the same framing for every controller.

The comparison places five genuine scene recordings beside a common tracking-error plot.
At video simulation time t, each view selects its latest acquired image at or before t,
and labels the actual frame time/error. The error plot uses independent physics truth
on the common 12 ms evaluation grid, including the exact 32 s endpoint. The public timeline
retains selected frame hashes, times and ages; each original image timestamp is checked
against the paused physics timestamp. The MP4 plays at 1× simulation time; the smaller
GIF preview plays at 2×. Paused recording overhead is excluded from the frozen benchmark
and does not demonstrate real-time operation or physical-flight performance.

## Reproduce

Use a new dedicated simulator; inspect other experiments before operating it:

```bash
.venv/bin/python scripts/launch_simulator.py --local-config .local/environment.json --settings configs/airsim-demo-settings.json --local-output .local/figure-eight-simulator
.venv/bin/python scripts/check_environment.py --local-config .local/environment.json --output .local/figure-eight-capabilities.json
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/capture_tracking_comparison.py --local-config .local/environment.json --exclusive-simulator --task-config configs/figure-eight-demo-task.json --output results/runs/my-figure-eight
.venv/bin/python scripts/render_tracking_comparison.py --source results/runs/my-figure-eight --output results/runs/my-figure-eight-publication
```

Wait for the scene to load before checking RPC capabilities. Both output directories must
be new. The capture command resets/arms only the selected simulator and finishes each run
paused/disarmed. Frame acquisition happens outside the recorded controller/loop timing.
Use `--task-config configs/tracking-demo-task.json` to reproduce the retained circle demo.
Reference derivative consistency, C4 endpoint conditions, distinct crossing tangents,
MPC component-bound compatibility and exact-duration figure-eight reanalysis are tested.

## Measured delivery

[32 s synchronized MP4](../results/examples/figure_eight_demo/comparison.mp4), [2× GIF preview](../results/examples/figure_eight_demo/comparison.gif), [trajectory/error figure](../results/examples/figure_eight_demo/trajectories_and_errors.png) and [all five results](../results/examples/figure_eight_demo/summary.json).

All five complete the common 32 s interval and pass the 0.1 m traversal RMS threshold. Traversal RMS uses the full 2–30 s figure eight, including both lobes and both centre visits. There are no recorded collisions, actuator saturation, evaluated speed/tilt violations or MPC fallback activations. All 886 MPC QP updates report solved. These are one nominal run per controller; no statistical ranking is claimed.

| Controller | Whole-run position RMS (mm) | Maximum position error (mm) |
| --- | ---: | ---: |
| PID | 23.471 | 74.211 |
| LQR | 3.913 | 12.347 |
| Geometric SE(3) | 0.892 | 2.275 |
| Geometric + force DOB | 0.398 | 1.087 |
| Constrained linear MPC | 4.128 | 13.030 |

All five recordings use the same clean source `3eb6692`; exact commits/configurations are retained in metadata and the [capture source check](../results/examples/figure_eight_demo/capture_source_check.json). Controller gains were not retuned. The rendering revision is recorded separately in the media validation.

Original raw images and logs remain in the local archive identified by [its size and SHA256](../results/examples/figure_eight_demo/full_archive.json). Compact data, full frame provenance, media and editable-source PDF/SVG/PNG figures are public. Recreating the scene video requires these local raw frames or a fresh simulator recording; no external upload of the large archive is claimed.

The [excluded preview record](../results/examples/figure_eight_demo/excluded_preview.json) preserves the original partial result and the reference-component-bound reason for stopping the earlier design. It is a planning abort, not a completed controller comparison.

Media checks confirm 1,605 actual scene frames with exact image/paused-truth timestamps and common fixed camera poses. The MP4 decodes to 320 frames at 1600×1008, 10 fps, 32 s. The 960×600 preview GIF decodes to 160 frames over 16 s; its compact palette explicitly reserves controller/marker colours. Maximum selected scene age is 0.1 s. The centre crossings occur at approximately 12.975 and 19.025 s and are marked on the error plots. The full 26-test suite, including the read-only AirSim integration, passes; lint and whitespace checks pass.
