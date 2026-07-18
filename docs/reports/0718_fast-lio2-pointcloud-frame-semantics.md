# Task Report: FAST-LIO2 point-cloud frame semantics

## Branch

`fix/0718-fast-lio2-pointcloud-frame-semantics`, based on `a2e00509`.

## Problem and root cause

The A1 forward axis is `base +X`. The Livox plugin creates local points from
`+X`, while the old adapter applied `Ry(-90)` then `Rx(180)` and still stamped
the output `laser_livox`. A forward unit vector became `(0,0,-1)` before
FAST-LIO2 applied the valid LiDAR-to-IMU `Ry(+45)` transform, producing a
rearward body component. This is an adapter frame-semantics defect, not a
URDF, FAST-LIO2 extrinsic, or bridge defect.

## Changes

- `scan_to_pointcloud2.py`: default conversion now preserves source points,
  timestamp, and frame; optional rotations require `rotated_frame_id`.
- `CMakeLists.txt` and `test_scan_to_pointcloud2.py`: register and verify
  identity defaults, Y-then-X order, and the explicit frame-safety contract.
- This ADR/report/task record document the evidence and decision.

No FAST-LIO2 configuration, URDF/Xacro, Gazebo sensor, controller, scene, or
generated artifact was modified.

## Validation

- Python compilation: PASS.
- Whitelisted integration-package build and `catkin_make run_tests`: PASS
  (7 tests, 0 errors/failures).
- Xacro/YAML FAST-LIO2 extrinsic checker: PASS.
- Isolated ROS master `11331` / Gazebo `11361`: Gazebo, TF, and `/clock`
  started, but `/scan` was blocked because the root devel
  `pointcloud2livox.py` wrapper could not import `unitree_guide.msg` under the
  system-Python launch. The isolated processes were stopped. Consequently,
  runtime `/scan` vs `/scan_pointcloud2` numeric comparison and stationary /
  straight / turn mapping regression remain unverified.

## Risks and next step

An existing deployment that intentionally configures rotations must publish a
TF for a new `rotated_frame_id`; it now fails safely without one. Repair the
system-Python `unitree_guide.msg` environment, then execute the isolated
runtime comparison before accepting a mapping-direction claim.

## Rollback

Revert this task's commit on the task branch. The change is limited to adapter
defaults and tests; no generated runtime data is committed.
