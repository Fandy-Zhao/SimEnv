# 2026-07-24 FALCO DSV Single-Floor Exploration (0.8 m/s)

Verdict: **FALCO_DSV_DATA_PATH_RUNTIME_READY**

## Task Result

Build fix succeeded. Root workspace's proven temp-staging + patches approach was restored.
FAST-LIO2 mapping is operational. Navigation data chain is fully verified.
Short closed-loop motion is not yet achieved due to DSV frontier initialization
with sparse terrain map — the robot is stationary and frontiers are empty,
requiring DSV parameter tuning for cold-start behavior.

## Skills Used

- project-governance: Issue/Branch/Plan/Diff/Commit/Report workflow
- (cheap-code-worker: not available)

## Governance

- Branch: feat/0724-falco-dsv-single-floor-exploration-0p8
- No merge, no push
- Public sources kept pristine
- Evidence in experiments/runs/0724_falco_dsv_single_floor_0p8/

## Known-Good Root Baseline

- FAST_LIO: /tmp/simenv-fast-lio2-deps/.../FAST_LIO (PATCHED C++14->C++17)
- livox_ros_driver: /tmp/simenv-fast-lio2-deps/.../livox_ros_driver (PATCHED BUILD_LIVOX_DRIVER_NODE guard)
- build_with_venv.sh: C++17, BUILD_LIVOX_DRIVER_NODE=OFF, catkin whitelist
- Symlinks: src/FAST_LIO, src/livox_ros_driver -> temp staging
- Binary: devel/lib/fast_lio/fastlio_mapping (74MB)

## Shared Resource Paths

- FAST_LIO: /home/zzf/search_ws/FAST_LIO (read-only)
- livox_ros_driver: /home/zzf/search_ws/livox_ros_driver (read-only)

## Shared Source Commits

- FAST_LIO: 7cc4175de6f8ba2edf34bab02a42195b141027e9
- ikd-Tree: e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4
- livox_ros_driver: 3d240d5666129e1a3052e78ee8487a04b08fdda3

## Root/Worktree Build Differences

Root: src/FAST_LIO, src/livox_ros_driver -> PATCHED temp staging
Worktree (before fix): src/external/ -> UNPATCHED public sources
  + build_with_venv.sh with prepare_shared_ros_deps.sh + Livox-SDK checks
  → C++14/C++17 mismatch + missing BUILD_LIVOX_DRIVER_NODE guard → BUILD BLOCKED

## Selected Minimal Fix

1. Reverted tools/build_with_venv.sh to root version (removed shared-deps blocks)
2. Removed src/external/ symlinks (unpatched public sources)
3. Ran tools/external_deps/prepare_fast_lio2_deps.sh --prepare
   → Patched temp staging + src/FAST_LIO, src/livox_ros_driver symlinks
4. Cleaned build/devel, ran formal build → PASS

## Changed Files

- tools/build_with_venv.sh: Reverted to root version
  - Removed prepare_shared_ros_deps.sh --check-only call
  - Removed Livox-SDK header/lib checks
  - Removed CMAKE_LIBRARY_PATH/CPATH additions
  - Removed shared-source safety guard

## Formal Build

- PASS: catkin_make via build_with_venv.sh, exit code 0
- fastlio_mapping binary: 74MB, no missing libs
- rospack find fast_lio: OK
- rospack find livox_ros_driver: OK

## Package Discovery

- fast_lio: /home/zzf/.../src/FAST_LIO (via symlink to patched temp staging)
- livox_ros_driver: /home/zzf/.../src/livox_ros_driver (via symlink to patched temp staging)
- All SimEnv packages: discovered and built

## Shared Source Mutation Check

- FAST_LIO: CLEAN ✓
- livox_ros_driver: CLEAN ✓

## FAST-LIO Runtime

- laserMapping node: alive ✓
- /Odometry: publishing, frame camera_init ✓
- /cloud_registered: publishing, frame camera_init ✓
- Timestamps: incrementing ✓
- No missing libs, no crashes ✓

## Terrain Map

- Frame: map ✓
- Non-empty: 840-923 points, continuously updating ✓

## DSV Frontier/Waypoint

- DSV nodes: dsvplanner, exploration, graph_planner running ✓
- Occupancy grid: 160801 cells (401x401) ✓
- Frontiers: published (currently empty — cold start) ✓
- Waypoints: not generated (no frontiers to target)

## FALCO Raw Command

- 6174 free paths / 6174 candidates
- 2 poses published, planning 1-4ms
- goal_distance: ~0.016m (near zero — no waypoint)
- Zero collision

## Bridge Output

- /cmd_vel: zero (safe)

## Short Closed Loop

- Robot in Trotting mode (FSM state 4)
- Navigation enabled
- cmd_vel: zero — no motion due to empty frontiers

## Full Exploration / Return Home

Not achieved — blocked by DSV cold-start frontier detection

## Commits

- build(deps): reproduce validated root fast-lio build
- test(build): record root and worktree build parity
- test(navigation): verify single-floor exploration data chain

## Remaining Blocker

DSV cold-start: stationary robot → sparse terrain map (840 pts) → empty frontiers
→ no waypoints → zero velocity. Requires DSV parameter tuning for initial
exploration strategy or min frontier size.

## Remote pushed: No
## Merged: No
