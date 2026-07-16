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

## Residual runtime checks

Topic rate, timestamp delta, five-minute ROS-time endurance, `map → body` TF
success ratio, motion axes, and RViz overlay remain NOT RUN. Static inspection
and Stage 0/1 evidence support the unchanged `camera_init → body` and
`map → camera_init` semantics, but do not replace this runtime evidence.
