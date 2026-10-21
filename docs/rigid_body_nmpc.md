# Full rigid-body NMPC and acados acceleration

The new `nmpc` controller uses the complete six-DOF rigid-body equations with quaternion attitude and measured common motor lag. It retains the shared physical thrust/moment allocator. The previous `mpc` remains a near-hover linear baseline. The previous 70-run report and its failures are preserved; its aggressive failure must not be interpreted as a limitation of MPC as a method.

See [model, cost, constraints and all tunable parameters](../src/iet_ch5/controllers/nmpc/README.md). No arbitrary 2 m/s speed or small-angle state cap is carried into the nonlinear OCP. Experiment safety bounds remain shared. Full rigid-body modeling does not imply that every aerodynamic or simulator effect is modeled.

[Completed measured evaluation, video and timing ablations](rigid_body_nmpc_results.md) retain failures and limitations.

## What accelerates computation

The [official acados feature guide](https://docs.acados.org/features/index.html) describes RTI and partial condensing; the [Python interface guide](https://docs.acados.org/python_interface/index.html) documents generated C solvers and wrapper choices. This implementation uses:

1. CasADi-generated C functions and derivatives, compiled before evaluated control.
2. Gauss-Newton SQP-RTI: one SQP step at each update rather than full NLP convergence.
3. HPIPM with BLASFEO and partial condensing, exploiting the multiple-shooting OCP structure.
4. Shifted state/input warm starts and QP warm start.
5. Precomputed analytic reference targets; online interpolation and parameter updates are timed.

RTI preparation and feedback run in the same call here; asynchronous AS-RTI and Cython wrapper speedups are not claimed. A measured replay compares full SQP, RTI, cold start and condensing with the same model/horizon/cost and feedback. Different methods return different optimization iterates, so timings must be read with residuals, command differences and closed-loop outcomes. A paused AirSim run does not establish real-time flight.

## Install without changing the host environment

Tested on Linux x86_64, Python 3.10, CMake and GCC. The optional build stays in ignored `.local`, including generated C/libraries and machine paths. No AirSim rebuild or global Python installation is needed.

```bash
uv sync --extra dev --extra plots --extra mpc --extra nmpc --locked
.venv/bin/python scripts/install_acados.py
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 IET_CH5_TEST_ACADOS=1 .venv/bin/pytest -q
```

The installer pins released acados **v0.5.4**, commit `d9d0ba2e8f127855119d38874af8a9570df1d8e5`, plus its exact HPIPM/BLASFEO submodules. This is the tested pin, not a claim to use the newest release. Upstream interface metadata reports 0.5.1; the recorded source commit is authoritative. The official Linux renderer v0.2.0 is checksum verified. Builds record compiler, native-library hashes and generation provenance. Missing/changed native binaries are rejected. Source and renderer/binaries are not redistributed in this repository.

`acados_template` is installed from that ignored checkout after `uv sync`; a later exact `uv sync` may remove the external editable package, so rerun the idempotent installer afterward. Optional model/native tests explicitly skip when their dependencies or opt-in flag are absent. The installer refuses a different or modified existing checkout rather than overwriting it.

## Evaluate through the common environment

Launch an exclusive capability-checked simulator as in the root README; only the selected owned instance is reset/armed. The new suite contains linear MPC, rigid-body NMPC and geometric control, on seven unchanged reference trajectories, twice (42 runs). Historical five-controller suites remain unchanged.

```bash
.venv/bin/python scripts/run_case_study.py list
.venv/bin/python scripts/run_case_study.py plan --suite configs/experiments/rigid_body_mpc.json --output .local/rigid-body-plan.json
.venv/bin/python scripts/run_case_study.py run --suite configs/experiments/rigid_body_mpc.json --local-config .local/environment.json --exclusive-simulator --output results/runs/my-rigid-body-matrix
.venv/bin/python scripts/run_case_study.py publish --output results/runs/my-rigid-body-matrix --destination results/runs/my-rigid-body-publication
.venv/bin/python scripts/plot_case_study.py results/runs/my-rigid-body-publication --output results/runs/my-rigid-body-plots
```

A separate `configs/experiments/nmpc_development_trajectory.json` is available for pre-freeze development. Initial transfer/spatial/development-course checks completed with the same default weights and no fallback before freezing the evaluation suite. These are integration/development evidence, not additional independent evaluation repeats. The evaluation reference/configs must not be retuned from observed results.

## Separate scene demonstration and solver replay

```bash
.venv/bin/python scripts/run_case_study.py demo --suite configs/experiments/rigid_body_mpc.json --controller pid lqr geometric mpc nmpc --trajectory agile_course --local-config .local/environment.json --exclusive-simulator --output results/runs/my-nmpc-demo
.venv/bin/python scripts/run_case_study.py publish --output results/runs/my-nmpc-demo --destination results/runs/my-nmpc-video
.venv/bin/python scripts/benchmark_nmpc.py --source results/runs/my-rigid-body-matrix/runs/038-nmpc-agile_course-nominal-s0-r0 --output results/runs/my-acados-replay --repeats 3
.venv/bin/python scripts/benchmark_nmpc.py --output results/runs/my-acados-replay --plot-only
```

Use the actual NMPC run folder recorded in the matrix journal (the numeric prefix follows matrix order). The replay reads actual physics feedback/applied inputs from a run, then compares solver variants offline; it is not new closed-loop AirSim flight. Repeats reuse the same measured workload, with per-variant preparation excluded and reported. Both native solver time and Python/controller-call time are recorded. Status/fallback, nonlinear residuals and command differences remain visible. The common demo renderer supports up to six methods without changing source scene images.

## Provenance and limits

The model/OCP is original Chapter 5 project work. acados, HPIPM and BLASFEO are pinned external BSD-2-Clause solver dependencies; CasADi is an unmodified LGPL-3.0 dependency. Their licences and citations are identified in the source manifest/notices. The separate renderer release has no explicit licence in its pinned source tree; it is used only as an externally downloaded build tool, and no renderer code/binary is redistributed.

State/input costs and the finite-horizon terminal weight do not establish stability, recursive feasibility or a global optimum. Quaternion norm drift, nonlinear residuals, failed QPs and geometric fallback must be reported; status 0 for RTI does not mean a converged NLP. Native build/reference-preparation cost is outside evaluated updates, never hidden as online work. Motor lag/density approximations and omitted drag/wind remain model mismatch. No real-time or physical-flight claim is made.

The committed acceleration package also includes [portable exact feedback inputs](../results/examples/acados_acceleration/REPLAY.md), allowing replay without a simulator or the full raw archive.
