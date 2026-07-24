# FALCO R3 Real-Data Diagnosis Summary

Date: 2026-07-24

Branch: `feat/0723-falco-dsv-navigation-integration`

Worktree: `/home/zzf/search_ws/SimEnv_worktrees/falco-dsv-navigation`

## Verdict

Overall verdict: `FALCO_POINTCLOUD_FILTER_BLOCKED`

- `GOVERNANCE_PASS`
- `BUILD_PASS`
- `R3_AUTONOMY_MODE_PASS`
- `R3_SIM_TIME_WINDOW_FAIL`: runtime was measured with `/clock`, but low RTF
  and rospy busy-spin prevented a clean 10 sim-s scripted collector window.
  Case A/B logs record before/after sim time and topic rates instead.
- `R3_WAYPOINT_SEMANTICS_PASS`
- `R3_POINTCLOUD_ALIGNMENT_PASS`
- `R3_OBSTACLE_FILTER_FAIL`
- `R3_LOCAL_PLANNER_PATH_FAIL`
- `R3_PATH_FOLLOWER_PASS`
- `R3_MOTION_GATE_PASS`
- `SCOPE_AUDIT_PASS`

## Findings

1. The first runtime attempt did not provide FALCO with real FAST-LIO2 input.
   The recovered FAST-LIO2 staging package was missing from `/tmp`, so
   `fast_lio` could build from stale artifacts but `rospack` could not launch
   it. After `tools/external_deps/prepare_fast_lio2_deps.sh --prepare` and a
   direct FAST-LIO2 relaunch, `/Odometry` and `/cloud_registered` were live.

2. `runtime_real_data.launch` defaulted to intermediate `/state_estimation` and
   `/registered_scan` relay topics. In this recovered run those relays were not
   alive, so the navigation namespace initially had no input. The launch now
   defaults directly to FAST-LIO2 `/Odometry` and `/cloud_registered`.

3. FALCO's waypoint topic is `geometry_msgs/PointStamped`, not
   `PoseStamped`. Publishing a `PoseStamped` causes a ROS type mismatch and
   localPlanner ignores the waypoint. The R3 waypoint was regenerated from live
   `/navigation/state_estimation` in frame `camera_init`, yaw about
   `-0.000746 rad`, target about 0.8 m ahead.

4. With real cloud and `checkObstacle=true`, Case A publishes only one path
   pose at `(0, 0, 0)` and raw FALCO TwistStamped remains zero.

5. With the same odometry and waypoint but temporary `checkObstacle=false`,
   Case B publishes a multi-pose path and raw FALCO TwistStamped is finite and
   nonzero (`linear.x` about `0.095 m/s`). Obstacle checking was restored to
   true afterward.

6. `/navigation/enabled=false` keeps `/cmd_vel` at zero even when raw FALCO
   TwistStamped is nonzero.

## RTF

RTF is not proven as the direct root cause. It is a strong timing companion:
Case B advanced from sim `33.580` to `36.074` during 90 wall seconds, and
Case A advanced from sim `41.658` to `43.482` during 60 wall seconds. The
direct functional cause of zero path is obstacle filtering under real cloud.

## Scope Audit

- Gazebo physics changed: no.
- Collision geometry changed: no.
- FAST-LIO2 core changed: no.
- FALCO vendor core changed: no.
- Trotting/RL core changed: no.
- DSV/R5/R6 run: no.
- R4 Trotting run: no.

## Commands

Formal build:

```bash
cd /home/zzf/search_ws/SimEnv_worktrees/falco-dsv-navigation
./tools/build_with_venv.sh
```

Reproducible R3 runtime:

```bash
cd /home/zzf/search_ws/SimEnv_worktrees/falco-dsv-navigation
GUI=false ENABLE_RVIZ=0 START_BUILDING_CONTROL=0 START_CONTROLLER=1 \
  ENABLE_FAST_LIO2=1 FLOOR_COUNT=1 ROOMS_PER_FLOOR=4 TERMINAL_BACKEND=tmux \
  ./auto.sh

roslaunch simenv_navigation_bringup runtime_real_data.launch \
  start_dsv:=false start_falco:=true start_bridge:=true
```

Do not enter R4 yet.
