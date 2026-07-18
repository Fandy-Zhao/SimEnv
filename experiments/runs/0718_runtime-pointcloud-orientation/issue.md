# Issue: runtime pointcloud orientation verification

Date: 2026-07-18
Branch: `fix/0718-runtime-pointcloud-orientation`
Worktree: `/home/zzf/search_ws/SimEnv_worktrees/runtime-pointcloud-orientation`

## Goal

Locate and fix the Gazebo runtime condition where pointclouds still appear along robot `-X` and tilted downward by 45 degrees, using runtime evidence rather than static source inference.

## Known Evidence

- The FAST-LIO2 pointcloud frame-semantics fix has been merged into `master`.
- Functional merge commit: `69ff34e7 merge: preserve fast-lio2 pointcloud semantics`.
- The IDE worktree may still be on `test/0718-g2-trotting-motion-baseline` with uncommitted G2 changes.
- The legacy adapter rotation `Ry(-90)` then `Rx(180)` maps sensor `+X` to `-Z`; with the URDF `Ry(+45)` mount this appears as robot `(-X,-Z)`.
- Current URDF mount: `laser_livox_joint` at `xyz="0.2 0 0.08"` and `rpy="0 0.785 0"`.

## Scope

- Runtime launch/overlay diagnostics for `auto.sh`, ROS package resolution, scan adapter process, `/scan`, `/scan_pointcloud2`, TF, and FAST-LIO2 output.
- Minimal startup-chain or diagnostic fixes needed to ensure runtime uses the corrected adapter code.
- Automated runtime smoke checks for active code path, commit, parameters, point equality, TF, and registered cloud orientation where available.
- Documentation of runtime evidence and residual risk.

## Non-Scope

- Do not modify the current IDE/G2 worktree.
- Do not modify Unitree controller, G2 baseline, scene generation logic, navigation, exploration, or competition logic.
- Do not modify URDF, FAST-LIO2 `extrinsic_R`/`extrinsic_T`, or map bridge before proving the adapter runtime path is correct.
- Do not rotate points while continuing to publish `frame_id=laser_livox`.

## Acceptance Criteria

- Runtime adapter is proven to come from a code path containing commit `69ff34e7`.
- `/scan` and `/scan_pointcloud2` timestamps and `frame_id` match.
- Sampled coordinates match point-by-point within `1e-6`.
- No explicit or implicit legacy `Ry(-90)+Rx(180)` rotation is active.
- The robot `base` frame no longer shows the legacy adapter-induced `-X` dominant direction.
- Any remaining 45 degree tilt is attributed to the URDF/TF mount, not an adapter rotation.
- `/cloud_registered` in `camera_init` is checked for horizontal ground and correct forward direction when FAST-LIO2 publishes it.
- The dirty G2 worktree remains untouched.

## Test Plan

- Record git worktrees, active branch/HEAD, merge-base status, `ROS_PACKAGE_PATH`, `CMAKE_PREFIX_PATH`, and `rospack find` results.
- Search launch, shell, YAML, and parameter files for legacy rotation parameters.
- Start Gazebo/ROS in the isolated worktree and record node/process evidence.
- Run adapter unit tests and Python compile checks.
- Run launch XML checks where available.
- Run a runtime smoke script against `/scan`, `/scan_pointcloud2`, TF, and `/cloud_registered`.

## Rollback

- Revert this task branch or remove the isolated worktree.
- No rollback action is needed for the IDE worktree because it is not modified by this task.
