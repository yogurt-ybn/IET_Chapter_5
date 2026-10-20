# M2 observer, MPC and disturbance acceptance

The fixed geometric nominal controller now has bounded **translational force-only** observer
compensation, and MPC solves a real constrained near-hover linear QP to produce the same
physical wrench as all baselines. No high-level AirSim flight command implements either.

## Observer specification and ablation

For constant mass, known inertial input `u = estimated_applied_f R e3 - m g e3`, the
low-pass disturbance-residual structure is `dhat_dot = lambda(m v_dot - u - dhat)`.
This is an original constant-mass specialization of Chen, Ballance, Gawthrop and O'Reilly,
“A nonlinear disturbance observer for robotic manipulators,” IEEE TIE 47(4), 932–938,
2000, DOI https://doi.org/10.1109/41.857974, section II-B. Primary author-hosted paper:
https://eprints.gla.ac.uk/320/1/gawthropiee474.pdf. No source code was copied and the
manipulator-specific nonlinear observer theorem is not asserted for this aircraft.

The implemented momentum-increment implicit-Euler form is

```
dhat[k] = (dhat[k-1] + lambda*m*(v[k]-v[k-1]) - lambda*dt*u_bar[k])/(1+lambda*dt)
```

`lambda=5 rad/s`; each force component is bounded to ±2 N. The observer uses an interval
average of telemetry-estimated applied collective thrust, not the requested wrench, with
the mean of consecutive feedback thrust directions. That is a quadrature approximation;
rotor telemetry's integration phase, motor dynamics, drag, modelling error and feedback
noise all affect the residual. The first update has zero velocity increment. For constant
disturbance and exact known input, the unbounded implicit-Euler error decays by
`1/(1+lambda*dt)` each step; bounds/noise/time-varying loads limit this ideal interpretation.

Compensation subtracts the estimated **inertial force vector** before deriving desired
attitude and scalar thrust. It is never subtracted from body moment. The estimate is held
constant in the geometric desired-attitude derivative calculation; no compensation-rate
feedforward is claimed. Nominal position/attitude gains are bit-for-bit the geometric gains.
The off ablation runs the same observer and nominal controller but disables its compensation.

Actual development wind runs both passed the hold criterion. Whole-run RMS was 0.062778 m
with compensation disabled and 0.030854 m enabled; sampled maximum error was 0.176100 m and
0.081000 m respectively. These two development runs do not establish a statistical ranking
and are separate from the frozen M3 matrix. Logs retain the initialization transient.

## MPC formulation

`controllers/mpc.py` uses the same 12-state zero-heading near-hover model as LQR, exact
ZOH discretization, design/optimization period 0.036 s, horizon 30 (1.08 s), and a held
wrench between QP updates. Inputs are full wrench deviations about `[mg,0,0,0]`. Stage
weights equal the documented LQR Q/W; terminal cost is its discrete Riccati P at the MPC
interval. There is **no terminal invariant set/constraint**, no robust tightening and no
claim of nonlinear stability or recursive feasibility. Motor lag is not a prediction state.

Condensed prediction is `X=Sx*x+Su*U`. The QP minimizes quadratic reference state deviation
and wrench deviation, with analytic 30-step preview of the shared planned task. At each
predicted step impose the exact shared allocation polytope
`0 <= inverse(B_allocation)*(equilibrium_wrench+delta_wrench) <= 1`.
This couples thrust and all moments. Speed components are bounded by `2/sqrt(3)` m/s and
roll/pitch components by `0.3/sqrt(2)` rad, conservative boxes for norm limits in the
small-angle model; they are not exact nonlinear tilt constraints. No position limit is
claimed. The common runner separately evaluates actual truth constraints and aborts.

OSQP 1.0.5 solves the sparse condensed QP, absolute/relative tolerances 1e-5, max 2000
iterations and solver time limit 8 ms, warm-started with the previous primal sequence
shifted one step (last input repeated); dual history remains the solver's automatic warm
start. Only solved status and maximum prediction violation <=2e-4 are accepted. Any other
status/timeout/violation invokes explicitly logged LQR fallback, then the same allocator.
Solver timing does not include Python QP-vector/preview work; both are recorded separately.
See https://osqp.org/docs/interfaces/python.html for upstream solver interface semantics.

The genuine AirSim development hover/transfer run passed: 0.006490 m whole-run position
RMS, 0.000082 m hold RMS, no fallback, no collisions and no rotor saturation. The cheap
held-output updates must not be used to suggest that the QP itself costs only a few µs.

## Explicit mechanism/failure checks

`scripts/check_m2_mechanisms.py` saved actual physics samples for:

* Wind at fixed hover motor inputs: positive x and negative y velocity changes under
  NWU `[8,-4,0]` m/s, with zero-wind control remaining horizontally stationary.
* An intentionally infeasible 100 N collective request: all four motors saturate,
  residual is reported, and actual normalized input is applied only in simulation.
* An actual OSQP time limit (1e-9 s): status `run time limit reached`, LQR fallback flag
  and resulting motor command are recorded and exercised in the simulator.

Numerical tests additionally force an infeasible predicted state and verify fallback,
check coupled input constraints, observer convergence/bounds, and exact disabled-nominal
equivalence. Shared feedback delay/noise has deterministic seeded-array/interpolation tests.

**Exact prescribed external force/moment pulse: unavailable.** The active server has no
such plant injection API. Wind is verified but is specified in m/s, not re-labelled N.
The available wind mechanism satisfies the documented disturbance scenario; a separate
server patch/build and open-loop force validation are required before a force-pulse claim.

M2's selected wind/ablation/MPC mechanisms passed development acceptance. M3 common-loop,
stress matrix and reproducible case-study interpretation still need measured evaluation.
