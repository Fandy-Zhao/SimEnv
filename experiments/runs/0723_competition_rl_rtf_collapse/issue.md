# Competition RL RTF Collapse

## Goal
Diagnose, with controlled variables, why
`WORLD_MODE=competition PHYSICS_PROFILE=normal GUI=true` plus the stair
RL policy correlates with RTF collapse and apparent LiDAR/PointCloud2/
FAST-LIO2/RViz mapping failure. The goal is causal attribution, not
parameter tuning or immediate optimization.

## Scope
- Competition-mode Gazebo simulation with Unitree A1 RL policy.
- Physics, sensor, mapping, and controller configurables in `auto.sh`.
- Wall-clock RTF, action inference latency, mapping-pipeline throughput.
- RTF matrix: nine flag combinations (M0-M8) controlled through
  `auto.sh` environment variables.
- Reproducible sampling of wall time, simulation time, RTF, process
  pressure, ROS nodes, topic availability, and mapping-chain health.

## Non-scope
- Optimization, retuning, or permanent launch default changes.
- Real-robot deployment or FreeDog hardware.
- Policy training or architecture changes.
- Competition scoring logic or danger-detection accuracy (except where
  RTF collapse makes detection infeasible).
- Long-term mapping drift (out of RTF-collapse regime).

## Acceptance Criteria
1. Root cause identified with evidence from runtime metrics (not
   static analysis alone).
2. The M0-M8 matrix documents RTF and key resource/mapping
   breakdowns.
3. Each conclusion distinguishes simulation-time evidence from wall-time
   observations.
4. Mapping-chain status is recorded for `/scan` ->
   `/scan_pointcloud2` → FAST-LIO2 → `/Odometry` /
   `/cloud_registered` → relayed `/state_estimation` /
   `/registered_scan`).

## Risks
- Adding diagnostics instrumentation could itself perturb RTF
  (mitigate: keep sampling lightweight, stdlib-only).
- Competition physics defaults (0.002/500/40) may be inherently
  incompatible with Torch inference at 50 Hz in the current hardware
  setup.
- Thread contention between Torch inference and FAST-LIO2 may be
  hardware-specific and not reproducible off-machine.

## Impacted Modules
- `auto.sh` — environment-variable dispatch and launch order
- `src/unitree_guide/…/State_RL_test.cpp` — policy load, inference
  thread, action buffer
- `src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py` —
  scan adapter
- `simenv_fast_lio2_integration/…/simenv_fast_lio2_mapping.launch` —
  FAST-LIO2 node and relay topics
- `tools/diagnostics/` — runtime sampling, mapping-chain inspection,
  RTF matrix harness
