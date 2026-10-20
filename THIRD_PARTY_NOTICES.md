# Source and licence provenance

The repository's original work has not been assigned a new blanket licence.
Consult the owner before redistributing original project code under new terms.
Upstream components retain their own licences; this file does not relicense them.

* **Microsoft AirSim** — Microsoft Corporation, MIT, pinned to
  `13448700ec2b36d6aad7a4e0909bc9daf9d3d931` plus pre-existing local telemetry
  extensions. See `docs/airsim_source_inventory.json`, `patches/airsim-dynamics-telemetry.patch`
  and `third_party/licenses/AirSim.txt`. The patch preserves the user's existing
  extensions, including upstream context. No Unreal binaries or scene assets are distributed.
  Rotor mapping and frame interpretation are derived from the installed source, not copied
  as a whole flight stack. The small RPC transport is an original implementation of its protocol.
* **FDCL uav_geometric_control** — Flight Dynamics and Control Lab, MIT, pinned to
  `f5fcb51c3b962152a4895a9d2e86e4a184655741`. Inspected C++ `position_control`,
  `attitude_control`, and `deriv_unit_vector`; these are the references for the
  standard SE(3) implementation. Its Python directory at this revision contains
  a dumbbell/spacecraft model and is not being treated as a drop-in quadrotor implementation.
  Normalization derivative algebra is adapted in `controllers/common.py`; the upward-frame
  moment law is implemented from the cited equations. Licence retained in `third_party/licenses/FDCL.txt`. Paper: T. Lee, M. Leok and
  N. H. McClamroch, “Geometric tracking control of a quadrotor UAV on SE(3),” CDC 2010,
  pp. 5420–5425, https://arxiv.org/abs/1003.2005.
* **safe-control-gym** — Dynamic Systems Lab, MIT, pinned to
  `6b5391d014f36fdfa0f9d22d92c77387e5274308`. Inspected PID, LQR and linear-MPC
  numerical implementations. Its Crazyflie PWM/RPM mapping is unsuitable for this AirSim
  vehicle, and the ROS/PyBullet/CasADi environment is not imported. Licence retained in
  `third_party/licenses/safe-control-gym.txt`. Source: https://github.com/learnsyslab/safe-control-gym.
  Paper: Z. Yuan et al., “Safe-Control-Gym: A Unified Benchmark Suite for Safe Learning-Based
  Control and Reinforcement Learning in Robotics,” IEEE RA-L, 2022.

NumPy, SciPy, MessagePack, OSQP, pytest and plotting packages are external dependencies,
installed separately with their upstream licences. Exact resolved versions are in `uv.lock`.
No upstream published performance is claimed to have been reproduced.

The force-only observer is an original constant-mass specialization of the residual structure
in Chen et al. (2000), DOI https://doi.org/10.1109/41.857974; no observer source code or
article text is redistributed. Its discrete implementation and limitations are specified
in docs/m2_acceptance.md. OSQP Python 1.0.5 is an external Apache-2.0 wrapper dependency,
pinned at `7065b3605925f725b7fa8ac76dc4d0bdbe512c75`; solver code is unmodified.
The MPC formulation/model/constraint assembly is original project code.
