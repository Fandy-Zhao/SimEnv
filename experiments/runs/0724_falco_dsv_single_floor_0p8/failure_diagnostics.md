# Failure Diagnostics

## 2026-07-24 Runtime Attempt

First failed gate:

`FAST_LIO_INPUT_BLOCKED`

Evidence:

- `auto_runtime.log` reports `fast_lio: NOT_FOUND`.
- `logs/fast_lio2.log` reports `ERROR: cannot launch node of type [fast_lio/fastlio_mapping]: fast_lio`.
- `rospack find fast_lio` fails after sourcing `/opt/ros/noetic/setup.bash` and this worktree's `devel/setup.bash`.
- `auto.sh` timed out waiting for `/Odometry`.
- The current `src/` tree contains `simenv_fast_lio2_integration` but no `fast_lio` package directory.

Action taken:

- Stopped navigation immediately.
- Did not enable `/navigation/enabled`.
- Did not command Trotting.
- Cleaned the ROS/Gazebo/tmux runtime processes started by this attempt.

## Runtime Scope

`auto.sh` + Gazebo was executed, but FAST-LIO2 did not start because the `fast_lio` package is not discoverable in this worktree. The run stopped before DSV start signal, navigation motion, short closed loop, full exploration, or return home.

## Gates Not Claimed

- `FALCO_DSV_DATA_PATH_RUNTIME_READY` not claimed.
- `SHORT_CLOSED_LOOP_PASS` not claimed.
- `FULL_EXPLORATION_PASS` not claimed.
- `RETURN_HOME_PASS` not claimed.
- Real-cloud terrain-map point statistics were not finalized; current parameters are initial candidates.

## Observations

- A synthetic cloud-only FALCO smoke proved the launch/bridge path but did not generate useful nonzero local-planner speed; direct path follower probes were used to validate speed scheduling.
- `registered_cloud_to_terrain_map.py` correctly refused to publish transformed terrain when no `camera_init -> map` TF existed in the synthetic smoke.
- Disabled bridge does not continuously publish zero if it has already published one zero; this avoids extra command spam but means `rostopic echo -n 3 /cmd_vel` can wait indefinitely in disabled state.
