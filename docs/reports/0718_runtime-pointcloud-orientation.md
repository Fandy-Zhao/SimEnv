# Task Report: Runtime Pointcloud Orientation

## Branch

`fix/0718-runtime-pointcloud-orientation`

## Summary

Runtime evidence does not support an active adapter rotation. In the isolated
worktree that contains `69ff34e7`, `/scan` and `/scan_pointcloud2` preserve the
same `laser_livox` timestamp, frame, and all sampled point coordinates exactly.
The observed 45-degree downward direction is the URDF physical mount
(`base -> laser_livox`), while FAST-LIO2 `/cloud_registered` is published in
`camera_init` with a fitted ground normal within `0.550 deg` of `+Z`.

The likely cause of seeing the old `(-X,-Z)` pattern in the IDE session is a
runtime path mismatch: the active IDE worktree was `a2e00509` and did not
contain the `69ff34e7` fix, the unsourced shell could not resolve SimEnv ROS
packages, and a stale root-worktree pointcloud relay process was still running
from `/home/zzf/search_ws/SimEnv/.venv`.

## Files Changed

- `auto.sh`: prints startup diagnostics for workspace path, branch, HEAD,
  whether `69ff34e7` is contained, ROS/CMake package paths, package resolution,
  and the adapter script path.
- `src/simenv_fast_lio2_integration/scripts/runtime_pointcloud_smoke_check.py`:
  new runtime checker for active adapter path, active git commit, rotation
  params, `/scan` vs `/scan_pointcloud2` coordinate identity, LiDAR TF, and
  optional `/cloud_registered` ground-plane normal.
- `src/simenv_fast_lio2_integration/CMakeLists.txt`: installs the runtime
  smoke checker.
- `experiments/runs/0718_runtime-pointcloud-orientation/`: issue, notes,
  worker inventory, and runtime logs.

## Runtime Evidence

- Adapter node: `/scan_to_pointcloud2_125132_1784388898756`
- Adapter PID: `125132`
- Adapter command:
  `/usr/bin/python3 /home/zzf/search_ws/SimEnv_worktrees/runtime-pointcloud-orientation/src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py`
- Adapter private params: `{}`; no `rotation_y_deg` or `rotation_x_deg`.
- `/scan` and `/scan_pointcloud2`: frame `laser_livox`, `24000` points,
  max coordinate error `0`.
- `base -> laser_livox`: translation `[0.2, 0.0, 0.08]`; LiDAR local `+X`
  maps to `[0.707388, 0.0, -0.706825]` in base.
- `/cloud_registered`: frame `camera_init`; fitted low-plane normal angle from
  `+Z` was `0.550 deg`.

## Coordinate Semantics

- `/scan`: raw Livox Gazebo plugin output in `laser_livox`; sensor local `+X`
  is the ray basis.
- `/scan_pointcloud2`: same coordinates and frame as `/scan`, only converted
  to XYZI `PointCloud2`.
- `/cloud_registered`: FAST-LIO2 registered map output in `camera_init`.
  The LiDAR 45-degree mount is handled by FAST-LIO2 extrinsic_R/extrinsic_T,
  not by the adapter or map bridge.

## Tests

- `python3 -m py_compile` on adapter and runtime smoke checker: pass.
- `xmllint --noout` on FAST-LIO2 and Gazebo launch files: pass.
- `catkin_make -DCATKIN_WHITELIST_PACKAGES=simenv_fast_lio2_integration`: pass.
- `runtime_pointcloud_smoke_check.py --timeout 20 --cloud-registered-timeout 10`: pass.
- `check_fast_lio2_extrinsics.py`: pass.

## Risks

- Full workspace build remains sensitive to optional local packages and Torch/CUDA
  environment; the task used a focused whitelist build.
- FAST-LIO2 itself came from the root worktree's untracked external
  `src/FAST_LIO` as a read-only overlay, while SimEnv integration and robot
  description were forced to the task worktree.
- Gazebo emitted repeated TF timestamp warnings from concurrent Gazebo truth and
  FAST-LIO2 TF publishers. They did not invalidate the adapter identity or
  registered-cloud plane checks, but are worth a separate TF ownership cleanup.

## Next Step

Run normal `auto.sh` from a branch containing this task after rebuilding the
workspace; its startup diagnostics should show the correct branch, HEAD,
package paths, and adapter script before mapping starts.
