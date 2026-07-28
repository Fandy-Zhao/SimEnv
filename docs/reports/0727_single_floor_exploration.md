# Single-floor exploration integration report

## Verdict

`SINGLE_FLOOR_EXPLORATION_BLOCKED`

The formal build passed and the runtime chain reached real robot motion, but a
complete exploration did not finish. Automated acceptance therefore blocked
the original merge. On 2026-07-28, the user explicitly authorized merging the
diagnostic fixes into local `master` with this blocked verdict preserved.

## First broken stages and repairs

1. Worktree dependencies originally pointed at raw shared FAST-LIO/Livox
   sources that did not build independently. The staging helper now consumes
   patched, isolated dependency sources.
2. Terrain filtering removed the mapped floor and DSV assigned wall height to
   unknown cells. Floor points are retained and single-floor unknown elevation
   uses the robot plane.
3. DSV consumed `camera_init` odometry as though it were in `map`, making every
   RRT candidate collide. `odometry_to_map.py` now composes the real static TF
   and transforms the pose.
4. Supervisor state was commanded through output topics and lacked the required
   `/navigation/exploring` output. `auto.sh` now uses request topics and the
   supervisor owns both exploration outputs.
5. Bridge and controller freshness mixed wall and simulation clocks. Both now
   use Gazebo time, including zero-stamped foot-contact messages.
6. A pure-yaw gait deadband of 0.20 rad/s rejected common FALCO commands below
   the 0.22 rad/s bridge cap. It now uses the 0.03 command deadband.

## Validation evidence

- Required build wrapper: PASS.
- Python syntax and 9 added unit tests: PASS.
- ROS master, Gazebo clock, stable robot, request/output state, FAST-LIO
  odometry/cloud, map-frame odometry, OccupancyGrid, DSV start/RRT, next goal,
  waypoint, FALCO path, path follower output, bridge gate, single `/cmd_vel`
  publisher, Trotting, and measured motion: observed.
- Final map: 200x360, 0.1 m/cell, 600 free, 7763 occupied, 63637 unknown.
- Final trajectory: 97 points, 0.24 m; robot did not fall.
- Final timing: 14.022 sim s, 463.149 wall s, average RTF 0.030275.
- Full exploration, explicit frontier clusters, reached goal, natural
  completion, and two additional post-fix repeats: FAIL/not completed.

## Governance decision

The dirty root worktree was backed up before task work, and all changes were
made on `test/0727-single-floor-exploration-artifacts` in the suffixed isolated
worktree. Blocking gates initially prevented the automatic merge. A later,
explicit user instruction authorized a local fast-forward merge despite those
known limitations. The pre-existing dirty root files remain protected and
nothing is pushed.
