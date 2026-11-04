# Portable measured feedback replay

The compressed feedback contains precisely the 438 measured control/physics-feedback records used per replay repeat, preserving all original fields. It is not new flight evidence. The matching original metadata is retained. Install the pinned optional acados environment, then from the repository root:

```bash
mkdir -p .local/acados-replay-input
cp results/examples/acados_acceleration/feedback_metadata.json .local/acados-replay-input/metadata.json
gzip -dc results/examples/acados_acceleration/feedback.jsonl.gz > .local/acados-replay-input/log.jsonl
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/benchmark_nmpc.py --source .local/acados-replay-input --output results/runs/my-portable-replay --repeats 3
.venv/bin/python scripts/benchmark_nmpc.py --output results/examples/acados_acceleration --plot-only
```

Numerical controller source is frozen at 9951545. Later commits add postprocessing/documents; no controller/model/configuration changes occur before publication. Timing varies by hardware/load. Full SQP and RTI are not equally converged NLP solutions; read residuals, statuses and command differences with timing. Execution hardware/thread settings and the feedback checksum are in execution_environment.json.

Verification on the recorded host: a fresh replay from these compressed records reproduced all four variants’ complete command arrays exactly (maximum absolute difference 0). Timing varied, as expected on a shared host.
