# Chapter 5 implementation handoff

Read `docs/IMPLEMENTATION_REQUIREMENTS.md` and `docs/UPSTREAM_CANDIDATES.md` before implementation. The user requested conventional UAV control code and an AirSim case study for the book, published in this repository.

The requirements are the implementation contract. Preserve the user's existing AirSim source modifications and local experiments. Use the chapter's upward inertial/positive-thrust convention and one verified thrust/moment-to-motor adapter for all controllers. Record source/licence provenance, actual simulator evidence and failures. Never replace a controller with a built-in AirSim high-level command under another name.

Proceed in M0, M1, M2, M3 order. Keep implementation in a dedicated directory/feature branch and push reviewable milestones to this repository. Do not invent experiment results or mark unfinished checkpoints complete. Keep local addresses, credentials, manuscript files, complete Unreal installations and proprietary assets out of public commits.

## Learning-control extension (Section 5.4.6)

The current learning task follows `docs/learning/REQUIREMENTS.md`. Read the measured
`docs/learning/CASE_STUDY.md`, `GP_NMPC.md` and `UPSTREAM_ASSESSMENT.md` in that
directory before changing learning code or running new experiments. The first case
compares the same rigid-body NMPC with a frozen force GP disabled/enabled; the older
linear MPC is not the nominal baseline for this ablation. Preserve flight-level
training/validation separation, model/data hashes and matched pair settings. Other
learning algorithms are assessed only; do not claim they have been ported or trained.
Current feature branch: `feat/ch5-learning-residual-nmpc`, draft PR #3.
