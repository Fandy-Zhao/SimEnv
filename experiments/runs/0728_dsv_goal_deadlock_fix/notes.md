# DSV goal deadlock repair notes

## Governance baseline

- Date: 2026-07-28
- Baseline local master: `8f56987b699d01124135d5f74e5430174862defd`
- Root workspace: `/home/zzf/search_ws/SimEnv`
- Root branch/HEAD: `master` / `8f56987b699d01124135d5f74e5430174862defd`
- Task branch: `fix/0728-dsv-goal-bootstrap-deadlock`
- Task worktree: `/home/zzf/search_ws/SimEnv_worktrees/dsv-goal-bootstrap-deadlock`
- Root dirty runtime artifacts were preserved and are outside the task diff.

## Captured failure evidence

- Local graph remained at 1 vertex; global graph remained at 1 vertex.
- RRT rejection diagnostics: Octomap rejected 997--1001 of about 1001
  extension attempts (99.6--100%).
- DSV goal was about 0.013--0.017 m from the robot.
- FALCO `stopDisThre=0.2 m`; raw command and `/cmd_vel` remained zero.
- Warm-up repeated `mode=2`, `valid_goal_count=3`, minimum 5 without a bounded
  termination path.
- FALCO collision candidates were not the blocker: 6174 candidates and 6174
  free paths were observed.

## Implementation decisions

- Supervisor owns exploration start (`autoExp=false`).
- Startup bootstrap is a fixed 1.2 m body-forward target, constrained to the
  initial floor, with >=0.5 m measured displacement and fail-closed timeout.
- Useful DSV goal threshold is 0.4 m: twice FALCO's 0.2 m stop distance and
  below DSV's 0.6 m minimum graph vertex separation.
- Planner-unready/premature completion gets three retries, one bootstrap
  recovery, then a clear failure rather than false completion.
- Octomap rejection is split into unknown and occupied counts; collision
  checking remains enabled and unchanged.

## Validation log

- Formal `./tools/build_with_venv.sh` passed before and after the Octomap
  collision-box calibration; logs are retained beside this file.
- First closed-loop run proved the bounded failure path: startup bootstrap
  displaced the robot 0.962 m, RRT still rejected 947--999 candidates as
  occupied, exactly one recovery bootstrap displaced it another 0.968 m, and
  the process exited with `BOOTSTRAP_EXPLORATION_FAILED` rather than looping.
- An exported occupied Octomap point cloud contained 1,020 floor voxels at
  `z=0.1 m`. With robot odometry near `z=0.35 m`, the old 0.35 m-high symmetric
  collision box sampled down to the same 0.20 m floor voxel. `kBoundZ=0.25 m`
  keeps the lower sample above that voxel without changing XY clearance or
  bypassing collision checking.
- Calibrated run: Stage 10 passed with `gate=closed`; Stage 11 then confirmed
  `trotting=true enabled=true exploring=true`. Startup bootstrap displaced the
  robot 0.962 m on the same floor. The first exploration RRT grew to 20 local
  vertices and a later cycle reached 31, with the global graph at 4 vertices.
  Graph Planner published a four-pose path ending near
  `(-4.041, 2.796, 0.349)` and FALCO plus `/cmd_vel` published non-zero angular
  commands. The A1 then exposed a pre-existing profile liveness conflict:
  `allowReverse=false` with `reverseEscapeEnabled=true` repeatedly changed the
  rear-goal heading by pi, alternating yaw commands and preventing translation.
  The profile now disables timed reverse escape so the existing turn-in-place
  gate can converge without changing any safety threshold.
- Final calibrated run (`runtime_final`) passed the short closed loop:
  supervisor gate closed before exploration, bootstrap displacement 0.963 m,
  local RRT 22 vertices, global graph 4 vertices, and a finite recorded DSV
  goal `(-4.079, 2.551, 0.343)` at sim time 13.46. Relative to the bootstrap
  completion pose `(0.100, 3.291, 0.342)`, the goal was about 4.24 m away.
- Graph Planner published four poses ending at that DSV goal and a non-zero
  waypoint. FALCO reported 6,174 candidates, 2,695--4,604 free paths, and
  `goal_dis=0.590--0.601 m`; `input_fresh=1`, `safety_stop=0`, and
  `reverse_escape=0`. `/cmd_vel` had the sole expected publisher
  `/cmd_vel_bridge` and was observed at linear 0.126 m/s and angular
  0.220 rad/s.
- State estimation reached `(-0.177, 3.944, 0.340)`, 0.709 m from the
  post-bootstrap pose, while remaining upright and on the same floor. The
  recorder captured a 1.95 m total route and the DSV goal event. Its summary's
  `Robot fall detected: True` is a known false positive from applying an
  absolute height threshold to FAST-LIO's `camera_init`-relative `/Odometry`
  (`z` near 0.02--0.05 m); map-frame state estimation stayed near z=0.34 m
  with near-zero roll/pitch and the robot continued commanded locomotion.
