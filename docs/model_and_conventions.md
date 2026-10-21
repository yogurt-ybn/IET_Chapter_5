# Model and convention contract

SI units throughout. World is NWU (north, west, up); body is FLU (forward, left, up).
`R` maps FLU body to NWU world. Positive scalar thrust acts along body +z.
`J` is inertia in FLU, and `W` denotes LQR/MPC input cost, never rotation.

```
p_dot = v
m v_dot = f R e3 - m g e3 + d_f
R_dot = R hat(omega)
J omega_dot = tau - omega cross (J omega) + d_tau
nu = [f, tau_x, tau_y, tau_z]
```

Position/velocity errors for translation feedback are desired minus actual. The SO(3)
error uses the standard explicitly negative-feedback form; linear controllers may use
actual minus reference. A force vector in NWU and its scalar projection onto `R e3`
are distinct quantities.

## AirSim transformations

Both world NED -> NWU and body FRD -> FLU use the proper rotation
`C = diag(1,-1,-1)`, determinant +1:

```
p = C p_ned                 v = C v_ned
R = C R_ned_frd C.T         omega = C omega_frd
force_nwu = C force_ned     tau_flu = C tau_frd
J_flu = C J_frd C.T         yaw_nwu = -yaw_ned
```

These are complete basis changes, not a z-only reflection. AirSim quaternions are named
`w_val,x_val,y_val,z_val`; SciPy input is explicitly `[x,y,z,w]`. Quaternion sign does
not change the rotation. `State` stores a rotation matrix instead of a quaternion.
`FastPhysicsEngine::computeNextPose` integrates `current_orientation * body_increment`;
its angular rate is body FRD. `AirSimSimpleFlightEstimator` passes that body rate through
and returns physics truth. The misleading `kinematics_estimated` name is not evidence
of sensor estimation. Main feedback uses atomic physics truth samples from the extension.

Unit tests cover nonzero attitudes, yaw, position/velocity, rate, force, moment, non-diagonal
inertia, cross-product preservation and round trips. Runtime small moment tests check signs.

## Rotor order and physical allocation

The RPC positional argument order is `[front_right,rear_left,front_left,rear_right]`,
also the active geometry order. In FLU the measured positions have horizontal signs
`[(+,-),(-,+),(+,+),(-,-)]`. All normals are +z. AirSim turning directions are
`[-1,-1,+1,+1]` (CCW=-1, CW=+1 in its source convention). Use the reaction-moment
vector from the source, rather than reinterpreting spin from a camera viewpoint.

For normalized motor input `u_i`:

```
u_filtered_dot = (u - u_filtered)/0.005
speed = max_speed * sqrt(u_filtered)
T_i = max_thrust * u_filtered * density_ratio
M_i = r_i cross (normal_i T_i)
      + normal_i * max_torque * direction_i * u_filtered * density_ratio
nu = B u_filtered
```

The simulator uses a discrete first-order filter plus update phasing; the differential
filter above is its continuous interpretation. `density_ratio` is active air density
divided by `EarthUtils::getAirDensity(0)`, whose float-rounding difference is negligible
and is checked against hover acceleration. Telemetry thrust/torque fields omit this final
density factor. The current published sample's rotor fields are sampled after pose
integration; `from_telemetry` is a model estimate with this phase limitation, not a direct
measured total wrench. Physics includes drag and collisions beyond the chapter model.

Steady input mapping is linear in thrust, not linear in rotor speed; do not square PWM.
Positive FLU moment signs across motors are roll `[-,+,+,-]`, pitch `[-,+,-,+]`, yaw
`[-,-,+,+]`. `Allocator.B` is derived from active geometry/reaction coefficients.
Infeasible requests use bounded weighted least squares on `0 <= u <= 1` with scales
`[1/(mg),1/0.3,1/0.3,1/0.1]`; the latter three denominators have units N m.
Always log the requested-minus-predicted residual and saturation flags. This estimated
steady achieved wrench differs from the dynamic telemetry estimate during motor lag.

## State and reference contract

`State`: `p[3]` m, `v[3]` m/s, `R[3,3]`, `omega[3]` rad/s, integer physics timestamp ns
and feedback provenance. `Reference`: desired p, v, a, jerk, snap in NWU and heading,
heading rate and heading acceleration in radians and seconds. Every controller resets
its memory before a run. Wrench is `f` N and body `tau[3]` N m. RunResult records status,
log path, metrics and failures; failed acceptance is never mapped to success.

M0 defines infrastructure only. Controller-specific models and equation mapping will be
added with their implementation milestones; no controller has passed acceptance at M0.
