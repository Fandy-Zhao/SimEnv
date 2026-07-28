# Plan

1. Record the `master` baseline and independent worktree identity.
2. Read every required startup, sensor, mapping, navigation, bridge, and governance file; map nodes, topics, services, states, waits, and forbidden depth-camera boundaries.
3. Capture baseline configuration and runtime/RTF evidence using the unified project launcher.
4. Implement the smallest event/state-driven startup change and permitted optional-load defaults.
5. Run syntax/static checks, the official build wrapper, runtime cases A-D, failure injections, topic/state evidence, and matched RTF/CPU measurements.
6. Prove depth-camera behavior is unchanged, update status/report documents, inspect the full diff, and commit only passing changes.
7. Fast-forward local `master` and rebuild/smoke-test the root workspace only if all required acceptance gates pass.

## Files to Read

- `AGENTS.md`, `PROJECT_STATE.md`, `CHANGELOG.md`, `docs/module_status.md`
- `auto.sh`, `tools/build_with_venv.sh`
- `src/unitree_guide/unitree_guide/unitree_guide/launch/multi_floor_gazeboSim.launch`
- `src/unitree_guide/unitree_ros/robots/a1_description/xacro/robot.xacro`
- `src/unitree_guide/unitree_ros/robots/a1_description/xacro/gazebo.xacro`
- `src/unitree_guide/unitree_guide/unitree_guide/scripts/pointcloud2livox.py`
- `src/simenv_fast_lio2_integration/launch/simenv_fast_lio2_mapping.launch`
- `src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py`
- `src/simenv_fast_lio2_integration/config/simenv_mid360.yaml`
- `src/navigation/simenv_navigation_bringup/launch/single_floor_exploration.launch`
- `src/navigation/simenv_navigation_bridge/scripts/nav_state_supervisor.py`
- `src/navigation/simenv_navigation_bridge/scripts/cmd_vel_bridge.py`
- Direct launch/config/script dependencies discovered from those files

## Expected Edits

- `auto.sh`
- The actual LiDAR visualization configuration file, only if the audited change is isolated from depth-camera behavior
- At least one of `PROJECT_STATE.md`, `CHANGELOG.md`, or `docs/module_status.md`
- Task evidence under `experiments/runs/0728_auto-event-driven-rtf-cleanup/`
- Final report under `docs/reports/`

## Explicitly Unmodified

- RealSense/depth-camera blocks and behavior
- Collision, physics, dynamics, controller/planner/mapping core algorithms
- Root-workspace generated data, logs, and results
