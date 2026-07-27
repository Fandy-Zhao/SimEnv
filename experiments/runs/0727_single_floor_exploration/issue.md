# Single-floor exploration validation issue

## Goal

Build the local `master` baseline through the governed build wrapper, run the
competition world as a complete single-floor exploration through `auto.sh`,
diagnose the first broken A-U navigation stage, apply only the minimum repair,
and preserve machine-readable exploration artifacts and timing evidence.

## Scope

- Gazebo, sensor, FAST-LIO2, map/Octomap, DSV, FALCO, path follower,
  `cmd_vel_bridge`, supervisor, `/cmd_vel`, Unitree controller, and actual
  motion runtime chain.
- State/goal/waypoint/frame/time topic consistency and startup ordering.
- Reusable artifact capture and directly related tests or diagnostics when a
  confirmed defect requires a code change.
- Governed commit, fast-forward local `master` merge, and post-merge build and
  runtime smoke only after all blocking acceptance gates pass.

## Non-scope

- Collision geometry, robot dynamics, core locomotion parameters, safety-gate
  removal, fabricated navigation messages, unrelated refactors, remote push,
  and deletion or overwrite of pre-existing worktree artifacts.

## Acceptance criteria

- Formal build passes via `tools/build_with_venv.sh`.
- A-U chain passes through actual robot motion and map growth.
- Full exploration completes naturally or is explicitly blocked at timeout;
  timeout is not success.
- Map, actual trajectory, exploration goals, timing, logs, and a topic-whitelist
  rosbag are preserved.
- Three startup checks establish repeatability.
- Only a validated, clean task commit may be fast-forwarded into local master.

## Risks

- Low Gazebo RTF may make 20-30 minutes of simulation time expensive in wall
  time.
- Existing ROS/Gazebo processes could contaminate topic ownership.
- External FAST-LIO2/Livox dependencies may not be independently available in
  the new worktree.
- The requested worktree path was already occupied by a dirty, unrelated
  branch, so this task uses the preserved `-0727` suffixed worktree.

## Expected impacted modules

Unknown until the first broken stage is confirmed. Candidate modules are
`auto.sh`, `src/navigation/`, FAST-LIO2 integration launch/configuration, and
experiment/status documentation. No candidate is authorization to edit before
runtime evidence identifies the root cause.
