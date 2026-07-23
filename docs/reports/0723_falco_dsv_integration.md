# FALCO + DSV-Planner Integration Report

Date: 2026-07-23

Branch: `feat/0723-falco-dsv-navigation-integration`

Baseline: local `master` `5bc0f6fbfdd8333dccbb44c26f216ecfb2811548`

## Summary

Imported FALCO `local_planner` and the minimum DSV-Planner package closure
under `src/navigation/vendor/`, then added SimEnv-owned bridge and bringup
packages under `src/navigation/`.

The integration keeps Gazebo, FAST-LIO2 core, robot model, Unitree controller
core, RL policy, physics, joystick, RViz, and rosbag launch behavior unchanged.
Launch adaptation is handled through namespace/remap files and a gated
`TwistStamped` to `Twist` bridge.

## Upstream Sources

- FALCO source: `/home/zzf/search_ws/ground_based_autonomy_basic`,
  branch `noetic`, commit `7ae94b72206430a399ae012f49715cf51fadb0e0`.
- DSV-Planner source: `/home/zzf/search_ws/dsv_planner`, branch `noetic`,
  commit `5ad721d243545cd0e6383ba2f3a4b9f218959ab4`.
- Vendored fallback ROS packages: `octomap_msgs` and `octomap_ros` from
  OctoMap `melodic-devel` sources because this host's apt sources do not
  provide the Noetic binary packages.

See `docs/navigation/THIRD_PARTY_SOURCES.md` for the full source list.

## Validation

- `python3 -m py_compile` passed for navigation bridge and smoke scripts.
- `tools/build_with_venv.sh` passed with the navigation whitelist.
- `rospack find` passed for imported/bespoke navigation packages.
- `roslaunch --files` passed for `falco_only.launch`, `dsv_only.launch`, and
  `falco_dsv.launch`.
- Static ROS interface checks found FALCO and DSV topics/services under
  `/navigation` without launching Gazebo, FAST-LIO2, robot models, joystick,
  RViz, or rosbag recording.
- FALCO smoke passed: synthetic odometry, cloud, and waypoint produced
  `/navigation/path`, `/navigation/falco/cmd_vel_stamped`, and gated `/cmd_vel`.
- DSV smoke passed: `/navigation/drrtPlannerSrv` and
  `/navigation/cleanFrontierSrv` were available, and synthetic inputs produced
  `/navigation/way_point`.

## Risks

- End-to-end Gazebo + FAST-LIO2 + DSV + FALCO + Unitree Trotting validation is
  still required before competition use.
- FALCO path assets are vendored as large upstream `.ply` files.
- The command bridge intentionally starts disabled and requires
  `/navigation/enabled=true` plus observed Trotting command state
  `/fsm/state_cmd==4`.
