# Shared-reference trajectory-tracking demonstration

The original body-relative geometric-controller video did not make trajectory tracking
visible. This replacement uses five independently recorded, genuine AirSim runs on exactly
the same predefined reference, initial state, measured plant, controller gains, feedback
and motor allocation. It is a separate visual demonstration, not a replacement for the
frozen 51-run benchmark or its timing measurements.

## Reference and view

The task configuration is `configs/tracking-demo-task.json`. Each run holds for 2 s,
traverses one horizontal circle of radius 1.5 m over 22 s, then holds for 2 s. The circle
starts at local NWU (0, 0, 2) m and has centre (-1.5, 0, 2) m, with fixed heading zero.
The degree-nine progress polynomial has zero first through fourth derivatives at the
entry and exit; it is not a constant-speed circle. The analytic reference's peak speed
is approximately 1.054 m/s and peak acceleration 0.741 m/s². No disturbance, parameter
mismatch or synthetic feedback noise is applied in this nominal demonstration.

A fixed external camera shows the entire circle. Yellow native AirSim debug markers
show the predefined reference; cyan markers show the already flown measured path.
The vehicle itself is the actual rendered simulator aircraft. All five scenes retain
the same framing. Camera position/orientation invariance is checked at each capture.

AirSim vehicle kinematics use the initial vehicle origin, while external cameras and
debug markers use global NED. The translation is measured from `simGetObjectPose(vehicle)`
minus `simGetVehiclePose(vehicle)`, rather than inferred from settings. This build's
spawn/ground adjustment means the observed translation differs from the configured
vehicle Z. It is recorded in each camera provenance file. The proper NWU-to-NED rotation
and this translation are both applied to markers and the camera.

The raw source attempt `results/runs/tracking-comparison-20261020` was interrupted during
PID after preview exposed the origin offset. Its partial logs are retained locally and
excluded from the delivered comparison. The aligned recordings use the corrected capture
source. All motion still comes from controller wrench requests and the common motor
bridge; camera/marker APIs never move the vehicle during evaluation.

## Reading the comparison

The video places five actual simulator views beside a common position-error plot.
The video clock is simulation time, not recording wall time. At presentation time t,
each view uses its latest captured image at or before t; each panel labels the actual
frame time and error. The plot uses independent physics truth resampled on the common
12 ms grid, including the exact 26 s endpoint. The delivered timeline records every
selected source frame, timestamp, age and SHA256. Raw images are not generated from a
trajectory plot. The GIF is a reduced-size, 2× speed preview of the same composed video; the MP4 plays at 1× simulation time.

The full simulated path spans 3 m; errors are shown separately in millimetres. This
keeps the physical path at its true scale while making small controller differences
readable. A successful nominal circle does not establish robustness, aggressive-flight
performance or a general ranking. The DOB uses the same geometric nominal gains, and
without a disturbance its result is expected to be close to the nominal controller.
The paused acquisition has substantial wall overhead and makes no real-time claim.

## Reproduce

Inspect other experiments and launch a dedicated simulator using the demo camera settings:

```bash
.venv/bin/python scripts/launch_simulator.py --local-config .local/environment.json --settings configs/airsim-demo-settings.json --local-output .local/tracking-demo-simulator
.venv/bin/python scripts/check_environment.py --local-config .local/environment.json --output .local/tracking-demo-capabilities.json
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/capture_tracking_comparison.py --local-config .local/environment.json --exclusive-simulator --output results/runs/my-tracking-comparison
.venv/bin/python scripts/render_tracking_comparison.py --source results/runs/my-tracking-comparison --output results/runs/my-tracking-publication
```

Wait for scene startup before the read-only capability check. The launcher refuses an
occupied port. Both output directories must be new. Capture resets/arms only the selected
exclusive simulator and disarms/pauses it after each run. Image acquisition is outside
control timing. The renderer checks common reference arrays, exact image/physics timestamps,
all image hashes, and final video decoding. Full original logs/images remain in ignored
local storage; the public package retains metric samples, original metadata/results,
frame provenance, video, GIF, representative images and editable-source PDF/SVG/PNG plots.
