# Summary

Final verdict: `FALCO_DSV_EXPLORATION_BLOCKED`

First failed runtime gate: `FAST_LIO_INPUT_BLOCKED`

The 2026-07-24 real run used the required entry points:

- `FLOOR_COUNT=1 GUI=false ./auto.sh`
- `roslaunch simenv_navigation_bringup single_floor_exploration.launch`

The run stopped before motion because `fast_lio` was not discoverable in this worktree, so `fast_lio/fastlio_mapping` did not start and `/Odometry` timed out.

## Completed

- Formal build passed with `./tools/build_with_venv.sh`.
- Added terrain-map and runtime-boundary adapters.
- Added `single_floor_exploration.launch`.
- Added FALCO heading-aware speed scheduling and diagnostics.
- Fixed DSV zero-initialization deadlock risk and replaced single-step movement threshold with windowed stuck detection.
- Tuned initial A1/single-floor DSV/FALCO/bridge parameters.
- Real runtime startup was attempted and failed at the FAST-LIO2 input gate.

## Validation

- `python3 -m py_compile`: PASS.
- `xmllint` launch XML: PASS.
- `roslaunch --nodes/--files simenv_navigation_bringup single_floor_exploration.launch`: PASS.
- FALCO interface smoke: PASS.
- FALCO path follower speed probes: PASS.
- Residual ROS process check after smoke: clean.

## Gate Verdicts

- `BASELINE_PASS`
- `SCOPE_AUDIT_PASS`
- `BUILD_PASS`
- `FALCO_SPEED_PROFILE_PASS`
- `FALCO_OBSTACLE_CHECK_PASS` for configuration/static preservation (`checkObstacle=true`); real-cloud obstacle runtime remains pending.
- `DSV_INIT_PASS` by build/static code path and parameter audit; full DSV runtime smoke with real planner not run.
- `DSV_MOVEMENT_DETECTION_PASS` by build/static code path and parameter audit; real stuck behavior pending.
- `TERRAIN_MAP_PASS` for node build/launch/interface; real cloud statistics pending.
- `SINGLE_FLOOR_CONSTRAINT_PASS` for config/code launch audit; real frontier/waypoint distribution pending.
- `BOUNDARY_PASS` for node build/launch/topic interface.
- `NO_MOTION_GATE_PASS` for bridge gating logic/static launch; disabled state does not continuously emit samples.
- `FAST_LIO_INPUT_BLOCKED`: first runtime failure.
- `TERRAIN_MAP_BLOCKED`: downstream of missing FAST-LIO2 input.
- `DSV_FRONTIER_BLOCKED`: downstream of missing FAST-LIO2 input.
- `FALCO_INTERFACE_BLOCKED`: downstream of missing DSV/FALCO real data path.
- `SHORT_CLOSED_LOOP_FAIL`: not executed.
- `FULL_EXPLORATION_FAIL`: not executed.
- `RETURN_HOME_FAIL`: not executed.

## Next

Restore or stage the `fast_lio` ROS package for this task worktree without modifying FAST-LIO2 core behavior, rebuild with `./tools/build_with_venv.sh`, and rerun the runtime gate from `/Odometry` and `/cloud_registered`.
