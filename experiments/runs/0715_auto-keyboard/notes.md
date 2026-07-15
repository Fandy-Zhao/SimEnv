# `auto.sh` keyboard validation notes

## Root cause

When FAST-LIO2 is enabled, `auto.sh` starts `junior_ctrl` asynchronously. In
a non-interactive Bash script, an asynchronous command without redirected
stdin receives `/dev/null`. The controller's `KeyBoard` thread reads stdin,
so no terminal key could reach it. `fg %1` is also not reliable without
interactive job control.

## Validation

- `bash -n auto.sh`
- Static assertions confirm `/dev/tty` is passed to background `junior_ctrl`
  and no `fg %1` remains.
- Full runtime launch intentionally not run: `auto.sh` kills existing
  Gazebo/ROS processes and regenerates the active scene.
