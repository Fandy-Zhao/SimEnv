# Notes: FALCO + DSV Integration

## Governance

- Baseline local `master`: `5bc0f6fbfdd8333dccbb44c26f216ecfb2811548`
- Task branch: `feat/0723-falco-dsv-navigation-integration`
- Task worktree: `/home/zzf/search_ws/SimEnv_worktrees/falco-dsv-navigation`
- Root workspace was dirty before the task; development was moved to this
  clean worktree.

## Cheap-Code-Worker

`cheap-code-worker` was invoked only for a mechanical source/dependency scan.
It could not access the external source directories and made no accepted
source changes. The integration, source audit, and interface decisions were
completed by the main agent.

## Current Verdicts

- Source import: `SOURCE_IMPORT_PASS`.
- Build: `BUILD_PASS` with `tools/build_with_venv.sh`.
- Launch parsing: `LAUNCH_PARSE_PASS` for `falco_only.launch`,
  `dsv_only.launch`, and `falco_dsv.launch`.
- Static interface: `STATIC_INTERFACE_PASS`; FALCO and DSV nodes come up
  under navigation namespaces without starting Gazebo, FAST-LIO2, robot models,
  joystick, RViz, or rosbag recording.
- FALCO smoke: `FALCO_INTERFACE_SMOKE_PASS`; synthetic odom/cloud/waypoint
  produced `/navigation/path`, `/navigation/falco/cmd_vel_stamped`, and gated
  `/cmd_vel`.
- DSV smoke: `DSV_INTERFACE_SMOKE_PASS`; `/navigation/drrtPlannerSrv` and
  `/navigation/cleanFrontierSrv` were available and synthetic inputs produced
  `/navigation/way_point`.

## Dependency Notes

- Installed host packages: `libgflags-dev`, `libgoogle-glog-dev`.
- `liboctomap-dev` was already present.
- `ros-noetic-octomap-msgs` and `ros-noetic-octomap-ros` were unavailable from
  this host's apt sources, so source copies of `octomap_msgs` and `octomap_ros`
  are vendored under `src/navigation/vendor/dsv/deps/`.
