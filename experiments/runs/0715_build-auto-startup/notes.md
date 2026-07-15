# Build and auto.sh Recovery Notes

## Goal

Make the documented build path produce every binary and plugin required by
`auto.sh`, and prevent a missing controller binary from causing a late,
misleading terminal exit.

## Diagnosis

- `logs/junior_ctrl.log` showed `junior_ctrl: No such file or directory`.
  The controller was not built; after its background process exited with 127,
  `auto.sh`'s foreground `wait` returned the same failure.
- A plain whole-workspace configuration also discovers the untracked external
  `ground_based_autonomy_basic/ps3joy` package.  Its optional `sixpair` tool
  requires the obsolete `libusb-0.1` pkg-config module, which is not supplied
  by this host's libusb-1.0 development package.  That external source is not
  part of this repository and is deliberately not changed by this task.

## Commands and results

| Command | Result |
| --- | --- |
| `bash -n auto.sh` | PASS |
| `./tools/build_with_venv.sh -j2` | PASS: selected seven-package runtime profile, including `livox_laser_simulation`, and completed `junior_ctrl` |
| `START_CONTROLLER=1 ./auto.sh` with `junior_ctrl` temporarily unavailable | PASS: exits 1 before cleanup, generation, or Gazebo; output saved in `auto_preflight.out` |
| `catkin_make --pkg unitree_guide -j2` | PASS: built `junior_ctrl` and `state_from_gazebo` |
| `catkin_make --pkg unitree_legged_control -j2` | PASS: built `libunitree_legged_control.so` |
| `catkin_make --pkg unitree_gazebo -j2` | PASS: built contact and draw-force Gazebo plugins |
| `source /opt/ros/noetic/setup.bash && source devel/setup.bash && /usr/bin/python3 -c 'from unitree_guide.msg import CustomMsg, CustomPoint'` | PASS |
| `START_CONTROLLER=0 ENABLE_FAST_LIO2=1 GUI=false ./auto.sh` plus `/gazebo/unpause_physics` | PASS: Gazebo publishes `/scan` at 10 Hz after the Livox plugin is built |
| Temporary adapter + FAST-LIO2 end-to-end smoke run | PASS: `/scan_pointcloud2` and `/Odometry` each publish at about 10 Hz |

## Residual validation

The initial six-package profile omitted `livox_laser_simulation`, so Gazebo had
no `/scan` publisher despite the IMU and FAST-LIO2 nodes being live. The
corrected profile builds `liblivox_laser_simulation.so`; the restarted Gazebo
session published `/scan` at 10 Hz, and a controlled adapter/FAST-LIO2 smoke
run restored `/Odometry` at about 10 Hz. Keyboard interaction remains a
separate interactive-terminal check.
