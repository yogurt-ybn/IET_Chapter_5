# acados acceleration: measured-state replay

Measured state/input replay, fixed model/horizon/cost/physical bounds; build/reference preparation excluded. RTI is one SQP step, not an equally converged NLP solution. No realtime-flight or generic solver superiority claim.

| Variant | Calls | Controller p95 (ms) | C solver p95 (ms) | Fallbacks |
|---|---:|---:|---:|---:|
| rti_warm_condensed | 1314 | 7.285 | 2.462 | 0 |
| sqp_warm_condensed | 1314 | 8.420 | 4.373 | 0 |
| rti_cold_condensed | 1314 | 6.737 | 2.233 | 0 |
| rti_warm_uncondensed | 1314 | 6.424 | 2.212 | 0 |

All variants use the same measured feedback records and reference, but compute different optimization iterates. RTI speed must be read together with residuals, command differences and closed-loop results. Cold-start and condensing ablations are implementation options, not independent controller families.
