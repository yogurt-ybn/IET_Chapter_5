# Frozen GP force residual inside rigid-body NMPC

The primary Chapter 5.4.6 case study compares the existing quaternion rigid-body NMPC with one frozen force-residual model disabled/enabled. It retains the 17 states, 40 × 24 ms horizon, 12 ms feedback, weights, terminal weight, rotor-input constraints, initial state, actuator lag, allocator and geometric fallback. The sole controller-setting difference within each pair is `nmpc.residual.enabled`.

## Residual definition and data alignment

The inertial acceleration predictor is

```text
v_dot = (f_applied R e3 + R F_GP(R.T v, f_applied)) / m_nom - g e3.
```

The GP predicts body-frame force discrepancy in newtons. Its four inputs are body velocity in m/s and telemetry-reconstructed applied collective in N. Quaternion, angular dynamics and wrench-lag states remain the existing nonlinear model. All three force axes are fitted independently. No moment residual is fitted.

Physics samples retain exact nanosecond simulation timestamps, NWU/FLU states and rotor telemetry. Labels use nonoverlapping, approximately 24 ms windows of the 3 ms physics stream:

```text
F_label = m_nom R_mid.T [ (v_end-v_start)/dt
                        - mean(f_applied R e3/m_nom - g e3) ].
```

Integrals use the recorded timestamps and trapezoidal quadrature; `R_mid` is the SO(3) midpoint between interval endpoint attitudes. Features are interval means. The initial 0.12 s is excluded from label construction as a declared reset/spin-up transient. Every physics timestamp/sequence is checked for missing or nonmonotonic samples. Collided/incomplete acquisition flights cannot become training data.

Applied thrust comes from measured filtered rotor state and the verified physical allocation, including the recorded density factor. It is a model estimate with known telemetry phase, not a load-cell measurement. Requested wrench is separately retained. Labels include unmodeled loads, integration and telemetry errors; they are **not measured wind force**. A unit test uses a known body force at a nonzero attitude and deliberately different commanded/applied thrust to verify the frame, sign and target.

## Flight split and model fitting

There are 10 actual acquisition flights, five per condition. Transfer, circle and helix flights train the models; separate figure-eight and spatial-course flights validate them. This is a whole-flight split with disjoint trajectory families, not a random split of adjacent samples. The final aggressive-course closed-loop reference is absent from training and prediction validation. Figure-eight closed-loop repeats are new flights of a previously validated reference family.

Nominal and mass-bias conditions have separate models. Mass bias means the controller assumes 1.1 times the verified plant mass; the AirSim vehicle and controller inertia are unchanged. This makes the learned discrepancy condition-specific. The learner never receives the true mass ratio as an input or a predefined correction formula.

The original implementation uses a standard RBF posterior mean with 64 support observations chosen by deterministic farthest-point sampling from training data only. Independent output length scales, amplitude and observation noise are fitted by bounded marginal-likelihood optimization. Scaling and output means use training data only. This is an exact GP on a subset of observations; it is not a variational sparse GP. Per-component force clipping is ±3 N and is represented identically in NumPy and CasADi. Posterior uncertainty is not used for chance constraints, safety certification or online adaptation.

The workflow is informed by [Torrente et al., Data-Driven MPC for Quadrotors](https://github.com/uzh-rpg/data_driven_mpc/tree/a6be5d301e66d91a823ef68a29c79ccc5829f1b9). That repository uses GPL-3.0-or-later, older ROS/Python dependencies and different model/input conventions. This project implements the standard GP equations and AirSim adapter in original code; it does not copy the GPL source, import its simulator, or claim to reproduce its published results. See the source assessment and notices.

## Independent prediction validation

Each held-out flight is checked at one prediction step (24 ms) and the complete 0.96 s horizon. Rollouts start from measured 17-state initial conditions, replay the recorded commanded-wrench sequence and freely integrate attitude, position, velocity and applied-wrench lag. No subsequent truth-state correction is applied. Both predictors use the same RK4 integration and quaternion normalization. Future recorded inputs are available in this offline diagnostic; this is not new closed-loop flight or a claim to know future control inputs online.

Starts are spaced 0.24 s apart, so horizons overlap. Their errors are diagnostic aggregates, not independent statistical replicates. The report records endpoint position/velocity errors, force-label fit error, complete error curves and model/data hashes. The measured mass-bias prediction improves substantially; nominal horizon prediction becomes slightly worse. All outcomes are retained. Validation is completed before the models are frozen for the paired flight matrix.

## Commands

Install the existing optional NMPC environment as described in the root README and `docs/rigid_body_nmpc.md`. No additional Python dependency is introduced. Use a dedicated, capability-checked AirSim instance; collection/comparison reset and arm only that selected instance.

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
.venv/bin/python scripts/run_learning_case_study.py collect \
  --local-config .local/environment.json --exclusive-simulator \
  --output results/runs/my-learning-collection
.venv/bin/python scripts/run_learning_case_study.py dataset \
  --source results/runs/my-learning-collection --output results/runs/my-learning-dataset
.venv/bin/python scripts/run_learning_case_study.py fit \
  --dataset results/runs/my-learning-dataset --output results/runs/my-learning-models
.venv/bin/python scripts/run_learning_case_study.py validate \
  --dataset results/runs/my-learning-dataset --models results/runs/my-learning-models \
  --output results/runs/my-learning-prediction
.venv/bin/python scripts/run_learning_case_study.py compare \
  --models results/runs/my-learning-models --validation results/runs/my-learning-prediction \
  --local-config .local/environment.json --exclusive-simulator \
  --output results/runs/my-learning-comparison
.venv/bin/python scripts/run_learning_case_study.py publish \
  --source results/runs/my-learning-comparison --dataset results/runs/my-learning-dataset \
  --models results/runs/my-learning-models --validation results/runs/my-learning-prediction \
  --output results/runs/my-learning-publication
```

Use new output directories. The separate learning comparison currently has no automatic resume; retain a failed/incomplete directory and use a new one for a changed experiment. Each run uses the common experiment engine and keeps infrastructure errors, safety aborts, solver failures and fallback visible. Within each condition/reference, repeat 0 runs off/on and repeat 1 runs on/off. This alternation does not make two repeats a statistical significance test.

Public compact dataset and frozen models allow fitting and prediction checks without Unreal. Plot commands also operate without a simulator:

```bash
.venv/bin/python scripts/run_learning_case_study.py prediction-plot \
  --source results/examples/learning_prediction --output results/runs/my-prediction-plots
.venv/bin/python scripts/run_learning_case_study.py plot \
  --source results/examples/learning_case_study --output results/runs/my-learning-plots
```

## Provenance and limits

`manifest.json` identifies flight splits, source revisions, exact model parameters, compact-file hashes and raw-log hashes. Model JSON files record training flights, parameters, optimizer status, input/output units and source fingerprints. Both paired variants receive the identical model path and SHA256; enabled dynamics verify its hash and nominal mass before code generation. Generated-solver cache keys include the model hash and GP implementation source. The model is frozen throughout evaluation.

This first case study does not evaluate online learning, uncertain state estimation, new wind conditions, model transfer across mass biases, hardware flight or real-time operation. It demonstrates one learned residual within the existing NMPC. Other supplied algorithm families receive source/interface/licence assessment before any separate adaptation or retraining.
