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
| `./tools/build_with_venv.sh -j2` | PASS: selected six-package runtime profile and completed `junior_ctrl` |
| `START_CONTROLLER=1 ./auto.sh` with `junior_ctrl` temporarily unavailable | PASS: exits 1 before cleanup, generation, or Gazebo; output saved in `auto_preflight.out` |
| `catkin_make --pkg unitree_guide -j2` | PASS: built `junior_ctrl` and `state_from_gazebo` |
| `catkin_make --pkg unitree_legged_control -j2` | PASS: built `libunitree_legged_control.so` |
| `catkin_make --pkg unitree_gazebo -j2` | PASS: built contact and draw-force Gazebo plugins |
| `source /opt/ros/noetic/setup.bash && source devel/setup.bash && /usr/bin/python3 -c 'from unitree_guide.msg import CustomMsg, CustomPoint'` | PASS |

## Residual validation

The live Gazebo session was not restarted because `auto.sh` intentionally
terminates existing simulation processes.  A subsequent clean interactive
startup is required to exercise keyboard input against the newly built binary.
