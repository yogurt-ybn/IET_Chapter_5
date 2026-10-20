# Chapter equation to code mapping

The requirements' preserved Chapter 5 equations and conventions are the implementation
contract. No manuscript is committed. Exact printed equation numbers have not been checked
against a typeset manuscript in this task; section mappings below do not invent them.

| Chapter concept | Implementation | Input/output and adaptation |
| --- | --- | --- |
| §5.2 rigid-body model | `frames.py`, `types.py`, `allocation.py` | SI, NWU/FLU, positive upward scalar thrust and body moments |
| §5.3 cascaded PID | `controllers/pid.py` | Position P -> bounded velocity, velocity PI + reference acceleration -> inertial force, SO(3) attitude P -> body rate, filtered rate PI -> physical moments |
| §5.3 near-hover state feedback | `controllers/lqr.py` | Full 12-state discrete aircraft model, altitude and yaw included; extends the illustrative planar pitch model |
| §5.3 SE(3) geometric tracking | `controllers/geometric.py`, `controllers/common.py` | Standard Lee/Leok/McClamroch moment law and FDCL direction derivatives, coupled yaw, upward thrust adaptation |
| §5.3.4 observer compensation | `controllers/dob.py` | Identical geometric nominal gains, bounded force-only momentum residual, off/on ablation |
| §5.3 constrained MPC | `controllers/mpc.py` | Real OSQP receding-horizon QP with coupled physical rotor feasibility |

Every controller implements `reset(state,reference,parameters)` and
`update(state,reference,dt) -> (WrenchCommand,diagnostics)`. None issues simulator flight
commands. The runner alone calls the shared motor adapter. Reference information is shared;
actual use differs and is stated below. Learning and physical deployment are outside scope.

## PID loops

All gains/limits are in `configs/controllers.json`. Position/velocity outer update period
is 36 ms (first update immediate). The inner attitude/rate loops update each measured control
sample, nominally 12 ms, subject to installed stepping jitter. The velocity PI produces
acceleration, with mass/gravity feedforward; horizontal and vertical components are each
bounded to 2.5 m/s². Desired attitude aligns body +z with that force and the prescribed
heading. Collective thrust is the nonnegative projection onto actual body +z.

Rate target limit is 2 rad/s; measured body rate has an 8 ms exponential low-pass filter.
Rate PI returns angular acceleration, premultiplied by J with gyroscopic compensation.
Velocity/rate integrals have explicit bounds. Allocation saturation freezes integrators;
acceleration clipping additionally back-calculates velocity-integral correction. This is
a modest near-hover cascade, not a port of PX4 or a renamed built-in AirSim autopilot.
It uses p/v/a and heading/rate; jerk/snap are available but unused.

## LQR model and design

State order `x=[p_x,p_y,p_z,v_x,v_y,v_z,phi,theta,psi,omega_x,omega_y,omega_z]`.
Input deviations `delta_nu=[f-mg,tau_x,tau_y,tau_z]`. Equilibrium is arbitrary constant
position, zero velocity/roll/pitch/yaw/rate and thrust mg. This first implementation uses
**zero equilibrium heading** and a near-hover envelope; it is not a yaw-scheduled model.
Roll/pitch/yaw are the FLU ZYX Euler components near zero, not globally nonsingular states.

Continuous nonzero blocks:

```
A[p,v]=I; A[vx,theta]=g; A[vy,phi]=-g; A[angles,omega]=I
B[vz,delta_f]=1/m; B[omega,tau]=inverse(J)
```

`hover_model` uses a block matrix exponential for exact ZOH discretization. Published design
period is 0.012 s. `Q=diag(8,8,12,4,4,5,12,12,8,0.3,0.3,0.3)` and
`W=diag(0.5,50,50,80)`. Solve the discrete Riccati equation once at reset (outside timed
updates), then `K=(W+B.T P B)^-1 B.T P A`. State deviation is actual minus reference,
and command is equilibrium/feedforward minus `K deviation`. This is explicitly negative
feedback. Design matrices and gain are saved in each run when available.

Reference roll/pitch are `(-a_y/g,a_x/g)`; rate references use jerk and moment feedforward
uses snap. Altitude feedforward is `m(g+a_z)`. Heading and heading derivatives are included
within the zero-heading neighborhood. Fixed gains tolerate the measured small sampling
variation but exact design-time stability does not prove variable-step nonlinear stability.
All motor constraints are handled by the common allocator; LQR itself is unconstrained.

## Geometric law

`F=m(a_d+g e3)+Kx(p_d-p)+Kv(v_d-v)` is an inertial force vector, while
`f=max(0,F dot R e3)` is scalar thrust. `Rd` aligns its third column with F and chooses
its first/second columns from heading with a right-handed orthonormal construction.
Analytic derivatives of F and of normalized directions provide Rd derivatives, desired
body rate and angular acceleration, using p/v/a/jerk/snap and heading derivatives.
Nominal dynamics are used for feedback derivative terms; drag/disturbance are not treated
as perfectly known.

```
eR = 0.5 vee(Rd.T R - R.T Rd)
ew = omega - R.T Rd omega_d
tau = -KR eR - Kw ew + omega cross (J omega)
      - J (hat(omega) R.T Rd omega_d - R.T Rd omega_d_dot)
```

The derivative normalization algebra is adapted from pinned MIT FDCL C++
`deriv_unit_vector`; standard SE(3) equations are cited in THIRD_PARTY_NOTICES.md. We do
not import its spacecraft Python example, its mixer, integral term or decoupled-yaw law.
Force norm below 1e-7 or a degenerate heading construction triggers a recorded fallback:
hold the last attitude for zero force, otherwise choose the least-parallel heading basis,
and set derivative feedforward to zero. Such fallback does not claim the original theorem's
regularity assumptions. The common near-hover experiment avoids these singularities.

Observer/MPC models, discretization, bounds, costs, preview, solver, fallback and primary
source citations are specified in [M2 acceptance](m2_acceptance.md).

## Additional figure-eight reference

`references/tasks.py` now supplies a C4 Gerono figure eight, with analytic derivatives
through snap from the first and second phase harmonics. The initial/final point is the
same verified hover state. Figure-eight duration and traversal windows propagate through
the shared runner, common-time reanalysis and plotting; it is not evaluated as a transfer
or silently truncated to the original 12 s task duration. See [reference equations and
validation](figure_eight_demo.md). Controller equations, gains and allocation are unchanged.
