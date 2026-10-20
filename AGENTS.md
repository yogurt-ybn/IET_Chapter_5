# Chapter 5 implementation handoff

Read `docs/IMPLEMENTATION_REQUIREMENTS.md` and `docs/UPSTREAM_CANDIDATES.md` before implementation. The user requested conventional UAV control code and an AirSim case study for the book, published in this repository.

The requirements are the implementation contract. Preserve the user's existing AirSim source modifications and local experiments. Use the chapter's upward inertial/positive-thrust convention and one verified thrust/moment-to-motor adapter for all controllers. Record source/licence provenance, actual simulator evidence and failures. Never replace a controller with a built-in AirSim high-level command under another name.

Proceed in M0, M1, M2, M3 order. Keep implementation in a dedicated directory/feature branch and push reviewable milestones to this repository. Do not invent experiment results or mark unfinished checkpoints complete. Keep local addresses, credentials, manuscript files, complete Unreal installations and proprietary assets out of public commits.