# Bridge registration race found during evaluation

The first frozen-matrix attempt at source `50ad612` was stopped and retained separately,
not pooled with replacement evaluation. It completed six runs (five tracking passes and
one registration failure) and was interrupted during the next run. Its original manifest,
summary and failed run's full control log/trace are in `results/examples/bridge_regression/`.
This is a bridge failure, not a controller performance failure.

The original bridge submitted a short motor request, allowed a fixed 1 ms scheduling
delay and advanced physics. Rarely the previous command's worker had not released its
single-task lock and the new passthrough goal did not reach the final sample of the step.
The strict check detected requested-versus-observed normalized differences around a few
millionths and aborted, rather than silently claiming the new command was applied.
Increasing numerical tolerance would not establish command ordering.

The corrected bridge uses the installed source's semantics: `moveByMotorPWMs` always
executes `commandMotorPWMs` once before entering its duration wait; `cancelLastTask`
cancels that wait without erasing the installed passthrough goal. While paused, submit
the new command, issue bounded cancellation until its RPC response arrives, and only then
advance physics. Repeated cancellation handles the worker starting after the first cancel.
Acknowledgement proves the command function ran. There is no join of an uncancelled future
that would require simulation-time progress. The unchanged strict motor telemetry check
still verifies the goal at the end of every physics step.

Motor registration acknowledgement, physics advancement and state-read overhead are
logged separately. Bounded socket/request timeouts remain in place. An unread response
from an interrupted RPC can now be discarded by request ID during cleanup; future IDs and
RPC errors are still rejected. Keyboard interruption retains the partial run and exits
after disarming/pausing its explicitly selected isolated simulator.

Following the correction, actual M0 hover/moment signs, pause, persistence/lag and batch
limit checks passed again, as did wind response, saturation and actual solver-timeout
fallback checks. Original ring-overflow source semantics are unchanged. The telemetry
source patch was independently applied to its clean upstream files and exactly reproduced
all 10 selected modified source files; this does not claim a new Unreal build was made.

No gains were changed in response to the abandoned evaluation. The replacement matrix is
frozen under a new source revision, with the original attempt available as failure evidence.
