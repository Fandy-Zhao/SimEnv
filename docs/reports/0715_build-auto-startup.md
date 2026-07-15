# Task Report — Build Profile and auto.sh Controller Guard

## Branch

`fix/0715-build-auto-startup`

## Summary

The reported terminal exit was caused by a missing `junior_ctrl` executable,
not by the terminal keyboard handoff: the controller background process exited
with status 127, then foreground `wait` returned that failure.  The supported
venv build profile now includes all packages needed by `auto.sh`, and the
startup script performs an early executable check before it cleans processes or
regenerates the scene.

## Changes

| File | Change |
| --- | --- |
| `tools/build_with_venv.sh` | Default whitelist now includes FAST-LIO2, `unitree_guide`, `unitree_legged_control`, and `unitree_gazebo`. |
| `auto.sh` | Checks `devel/lib/unitree_guide/junior_ctrl` before cleanup; reuses the checked path when launching the controller. |
| `README.md`, `docs/quick-start.md` | Make the supported runtime-profile build command the documented default. |
| Project status documents | Record diagnosis, build profile, and startup behavior. |

## Validation

- `./tools/build_with_venv.sh -j2`: pass; completed the six-package runtime
  profile and `junior_ctrl` target.
- `bash -n auto.sh`: pass.
- Controller-absent preflight: pass; it exits before mutating generated scene
  or running processes.
- `unitree_guide`, `unitree_legged_control`, and `unitree_gazebo` targets:
  built successfully.
- Unitree message imports from the sourced workspace: pass.

## Risks and next step

Plain `catkin_make` continues to discover untracked external packages in this
workspace and may require dependencies unrelated to SimEnv.  Use
`./tools/build_with_venv.sh` for the supported simulation runtime.  On the next
clean interactive run, execute `./auto.sh` and confirm `2`, `4`, and `6` reach
`junior_ctrl` through the same terminal.
