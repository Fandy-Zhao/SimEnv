# FAST-LIO2 Stage 2 Notes

## Baseline

- Branch: `feat/0717-fastlio2-stage2`
- Base: `818ee58e`
- Initial worktree: clean
- Design: transparent `topic_tools` relays; no message/frame rewriting; legacy
  topics and the existing static `map → camera_init` bridge retained.

## Results

- PASS: Python syntax for the bridge and contract test.
- PASS: three static launch-contract tests.
- PASS: launch XML and `roslaunch --files` resolution with the external
  FAST_LIO package exposed read-only through `ROS_PACKAGE_PATH`.
- PASS: package-scoped `catkin_make -j --only-pkg-with-deps
  simenv_fast_lio2_integration` in the Torch-enabled venv.
- PASS: isolated ROS relay smoke test under the shared lock. Source and
  destination types matched; relayed odometry retained `camera_init/body` and
  registered scan retained `camera_init`.
- BLOCKED: full-workspace Torch build reaches `unitree_guide`, then CUDA
  compiler identification fails because `nvcc` cannot execute `cc1plus`.
- BLOCKED: five-minute Gazebo validation was not completed. The first two
  attempts were terminated before topic capture: the first because
  `auto.sh` stale-process matching killed the lock wrapper (the lock path
  contains `gazebo`); the second was stopped on discovering it had launched
  `auto.sh` from the main worktree rather than the isolated worktree.

## Isolation incident

The stopped second attempt may have regenerated the main worktree's tracked
`generated_building/*`, `logs/building_control.*`, `logs/competition_gazebo.*`,
and `results/danger_truth.json`. Those paths were already dirty before this
task, so they were not restored, staged, or committed. No external FAST_LIO or
FALCO source was modified. Runtime must be repeated from an isolated worktree
whose own full build and generated/log paths are ready.

## Status before the final overlay run

Topic rate, timestamp delta, five-minute ROS-time endurance, `map → body` TF
success ratio, motion axes, and RViz overlay were NOT RUN at that point. Static inspection
and Stage 0/1 evidence support the unchanged `camera_init → body` and
`map → camera_init` semantics, but do not replace this runtime evidence.
The final overlay run below supersedes the topic-rate and TF gaps; motion-axis
testing and a rendered RViz screenshot remain outside this stationary Stage 2
validation.

## Isolated worktree runtime attempts

Two bounded attempts ran `stage2/auto.sh` with generated scene and log paths
inside this worktree while reusing main-worktree binaries read-only:

1. Gazebo reached ROS time 2.518 s, but every Unitree joint controller failed
   because pluginlib could not find the library for
   `unitree_legged_control/UnitreeJointController`; `/scan` was unavailable.
2. After explicitly adding the external built library directory, `/scan`
   became `sensor_msgs/PointCloud` and `/trunk_imu` reported z acceleration
   `9.79999999216685 m/s²`. However the controller remained at
   `Waiting for Gazebo joint state feedback before accepting stand command`,
   while controller_spawner still reported the same missing plugin. The run was
   stopped before FAST-LIO2 startup and no data was accepted as Stage 2 runtime
   evidence.

The root cause is the partial worktree devel overlay: rospack resolves the
worktree's Unitree package metadata, while its controller plugin was not built
because the full Torch/CUDA build is blocked. Environment-only library path
injection was insufficient for pluginlib resolution.

## Final 150-second runtime

The final isolated run reused the fully built `trot-rl/devel` only as a
read-only runtime overlay and kept scene/log generation in this worktree.
Pluginlib resolution was repaired with temporary devel-space symlinks, the
controller entered FixedStand, and upright gravity was measured at
`9.747037382398663 m/s²` (later `10.019715024621117 m/s²`).

The accepted ROS-time window reached exactly 150.0 s with 1500 odometry and
1500 registered-cloud messages (10.0 Hz each). Types and frames were
`nav_msgs/Odometry camera_init→body` and
`sensor_msgs/PointCloud2 camera_init`. The run exposed that FAST-LIO2 did not
broadcast its Odometry pose on TF, so `odometry_tf_bridge.py` was added and
validated live. `map→body` lookup succeeded 1197/1501 times; the 304 failures
are the startup interval before that bridge was introduced, not later drops.
The snapshot sample stamp delta was 0.212 s. A visual RViz screenshot was not
captured; TF connectivity and matching frames are the alignment evidence.
