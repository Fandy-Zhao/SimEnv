# 2026-07-25 DSV-Planner single-floor closed-loop smoke test

## Scope

- Worktree: `/home/zzf/search_ws/SimEnv_worktrees/dsv-single-floor-auto`
- Branch: `test/0725-dsv-single-floor-auto`
- Baseline HEAD: `d9bbe32394bf12e9094ff4b0264e2cfacdffecd5`
- Runtime entry point: `./auto.sh`
- Build entry point: `./tools/build_with_venv.sh`
- Scenario: `FLOOR_COUNT=1 ROOMS_PER_FLOOR=4 GUI=false ENABLE_RVIZ=0 ENABLE_NAVIGATION=1 NAV_MODE=dsv_falco NAV_AUTO_TROTTING=1 NAV_AUTO_ENABLE=1 NAV_AUTO_START_EXPLORATION=1`

## Result

Verdict: `DSV_PLANNER_OUTPUT_BLOCKED`.

The single-floor DSV/FALCO stack builds and starts through the real `auto.sh`
path. FAST-LIO2 publishes live `/Odometry` and `/cloud_registered`, navigation
bringup starts DSV, FALCO, graph planner, terrain adapter, boundary publisher,
and cmd_vel bridge, and the final Gazebo unpause check confirms `/clock`
advancing.

The first failed acceptance gate is T3 planning output: the quick 6 minute
runtime window did not confirm a DSV next-goal or FALCO velocity command on:

- `/navigation/dsv/next_goal`
- `/navigation/falco/cmd_vel_stamped`
- `/cmd_vel`

DSV was nevertheless active and repeatedly updating octomap output in
`logs/navigation.log`.

## Fixes made during test

- `auto.sh`: fixed timestamp readiness parsing for `/Odometry` and
  `/cloud_registered` by reading both `secs` and `nsecs`.
- `auto.sh`: fixed final Gazebo unpause verification to compare full
  `(secs,nsecs)` clock stamps instead of only integer seconds.
- `tools/build_with_venv.sh`: added the prepared Livox-SDK prefix and explicit
  `LIVOX_SDK_LIBRARY` handoff to avoid unsafe livox fallback discovery.
- `tools/build_with_venv.sh`: added `building_generator_interfaces` to the
  formal whitelist so door/elevator control service imports are generated.

## Evidence

- Formal build passed:
  `experiments/runs/0725_dsv-single-floor-auto/build_with_venv_staged.log`
- Incremental build after script fixes passed:
  `experiments/runs/0725_dsv-single-floor-auto/build_with_venv_after_wait_fix.log`
- Final runtime log:
  `experiments/runs/0725_dsv-single-floor-auto/auto_dsv_single_floor_final.log`
- Final runtime node/topic snapshot:
  `experiments/runs/0725_dsv-single-floor-auto/nodes_final_runtime.txt`
  and `experiments/runs/0725_dsv-single-floor-auto/topics_final_runtime.txt`
- Final navigation log tail:
  `experiments/runs/0725_dsv-single-floor-auto/navigation_log_tail_final.txt`

Key runtime observations:

- `[READY] topic: /Odometry (timestamps incrementing)`
- `[READY] topic: /cloud_registered (timestamps incrementing)`
- `Nodes: DSV=true  FALCO=true  Bridge=true`
- `[GAZEBO_FINAL_UNPAUSE] PASS: /clock advancing (5.122000000 -> 5.228000000)`
- `/Odometry` sampled near 10 Hz during the final runtime evidence window.
- Navigation topics included `/navigation/dsv/next_goal`,
  `/navigation/falco/cmd_vel_stamped`, `/navigation/terrain_map`, and
  `/cmd_vel`.

## Risks and follow-up

- The quick smoke window did not prove actual robot motion because no velocity
  command sample was received from FALCO or the bridge.
- `logs/navigation.log` still contains repeated PCL warnings:
  `Failed to find match for field 'intensity'.` This should be investigated in
  the registered-cloud-to-terrain-map path.
- Runtime cleanup through timeout did not fully stop all ROS children; this run
  required PID-targeted cleanup for the task worktree.
- The current reproducible build path depends on prepared staged FAST-LIO2 and
  livox message-only links created by
  `tools/external_deps/prepare_fast_lio2_deps.sh --prepare`.
