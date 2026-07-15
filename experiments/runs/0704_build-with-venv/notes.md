# Experiment Notes: 0704 build-with-venv

## Goal

Add `tools/build_with_venv.sh` for consistent catkin workspace builds using the project `.venv` Python.

## Checks Performed

| Check | Result |
|-------|--------|
| `bash -n tools/build_with_venv.sh` | PASS |
| `test -x tools/build_with_venv.sh` | PASS (executable OK) |
| `.venv` exists | YES (pre-existing) |
| ROS Noetic available | YES (`/opt/ros/noetic/setup.bash`) |

## Script Behavior

- Checks ROS Noetic setup, venv Python, venv activate, and `src/` directory before running.
- Exits with clear error message if any prerequisite is missing.
- Passes extra arguments through to `catkin_make` via `"$@"`.
- Sources `devel/setup.bash` at end (script-local only; user must source manually in their shell).

## Notes

- `.venv` already existed in the repo before this task; no venv was created.
- torch is not installed in `.venv`; the script will still build non-torch-dependent packages.
- `shellcheck` was not available; residual risk is low (script is straightforward).
