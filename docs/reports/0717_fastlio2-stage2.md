# Task Report — FAST-LIO2 Stage 2 Navigation Topics

## Branch

`feat/0717-fastlio2-stage2` from `master@818ee58e`.

## Summary

Added compatibility-preserving navigation outputs:

- `/state_estimation` (`nav_msgs/Odometry`) transparently relays `/Odometry`.
- `/registered_scan` (`sensor_msgs/PointCloud2`) transparently relays
  `/cloud_registered`.

The original FAST-LIO2 topics remain available. Relays preserve serialized
message stamps, covariance and frame fields. The existing bridge remains the
sole publisher responsible for `map → camera_init`; FAST-LIO2 retains
`camera_init → body`.

## Files Changed

- Integration launch: adds configurable navigation topic args and two
  `topic_tools/relay` nodes.
- Package metadata/README: declares `topic_tools` runtime dependency and the
  compatibility/frame contract.
- Static bridge remains on original `/Odometry`; a new dynamic Odometry TF
  bridge publishes `camera_init → body` from `/state_estimation`.
- Contract test: checks defaults, transparent relays, legacy-topic retention,
  and bridge input responsibility.
- Governance records and status documentation: record scope and evidence.

## Tests

| Check | Result |
|---|---|
| Python `py_compile` | PASS |
| Launch contract unit tests | PASS, 4/4 |
| `xmllint` | PASS |
| `roslaunch --files` with external package path | PASS |
| Package-scoped Torch/venv `catkin_make -j` | PASS |
| Locked synthetic ROS relay smoke | PASS: types and `camera_init/body` fields preserved |
| Full-workspace Torch build | BLOCKED: `nvcc` could not execute `cc1plus` |
| Isolated ROS-time Gazebo run | PASS for revised 150 s target; 1500/1500 messages |

## Runtime and Isolation

A runtime attempt was stopped when it was found to be executing the main
worktree's `auto.sh`. It may have rewritten already-dirty generated scene,
Gazebo/building-control logs and danger-truth result paths in that worktree.
Those user-owned dirty paths were not restored or staged. A prior attempt was
terminated by the launcher's stale-process cleanup matching the required
`/tmp/simenv-gazebo.lock` wrapper command line.

At that stage, the requested five-minute ROS-time rates, stamp deltas, TF
success ratio, motion-axis checks and RViz overlay could not be claimed. The
later final overlay run below supersedes the topic/TF gaps for the revised
150-second target. Motion-axis checks and an RViz screenshot would still require
another run
from a fully built isolated worktree, with all generated and log paths confined
to that worktree, while the whole Gazebo session holds the shared lock.

## Documentation Updated

Integration README, `PROJECT_STATE.md`, and experiment records.

## Git

No merge or push performed from this branch. See final task handoff for the
commit hash and diff stat.

## Risks

- `topic_tools/relay` adds one publish/serialization hop.
- The completed runtime target is 150 s rather than the original five minutes.
- The captured sample stamp delta was 0.212 s; full-window stamp-delta extrema
  were not persisted when the accepted snapshot was taken.
- The estimator child frame remains FAST-LIO2 `body`; downstream consumers
  must treat it as the estimator's IMU/body base or provide their own standard
  `base_link` transform.

## Next Step

For a longer endurance qualification, repair the CUDA host compiler lookup and
repeat the original five-minute matrix. The revised 150-second Stage 2 target
is complete.

The later isolated-worktree retry reached 2.518 s ROS time. A second startup
confirmed `/scan` as `sensor_msgs/PointCloud` and upright IMU gravity
(`z=9.79999999216685 m/s²`), but the Unitree controller never received joint
feedback. Gazebo reported that pluginlib could not find the library for
`unitree_legged_control/UnitreeJointController`. FAST-LIO2 and the five-minute
capture were deliberately not started, so this does not count as Stage 2
runtime acceptance.

## Final 150-second Validation

The final run used `trot-rl/devel` only as a read-only binary overlay and ran
`stage2/auto.sh`, generated scenes and logs entirely inside this worktree.
FixedStand and upright IMU gates passed before FAST-LIO2 startup.

| Metric | Result |
|---|---|
| ROS duration | 150.0 s |
| `/state_estimation` | 1500 messages, 10.0 Hz, `nav_msgs/Odometry` |
| Odometry frames | `camera_init → body` |
| `/registered_scan` | 1500 messages, 10.0 Hz, `sensor_msgs/PointCloud2` |
| Cloud frame | `camera_init` |
| Sample stamp delta | 0.212 s |
| `map → body` TF | 1197/1501 lookups (79.75%) |

The run identified a missing dynamic TF: FAST-LIO2 populated
`Odometry.header.frame_id/child_frame_id` but did not reliably broadcast that
pose. The new `odometry_tf_bridge.py` broadcasts the same finite Odometry pose
as `camera_init → body`. All TF failures occurred in the startup portion before
the live bridge was added; successful lookups then increased with every sample.
No RViz screenshot was captured, so alignment evidence is the connected
`map → camera_init → body` TF chain plus matching cloud/Odometry frames.
