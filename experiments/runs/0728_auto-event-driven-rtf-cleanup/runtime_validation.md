# Runtime validation

## Environment

- Branch: `refactor/0728-auto-event-driven-rtf-cleanup`
- Baseline commit: `e2e0e4ec241f100b7873f318a0b4e28944d7e65e`
- Entry point: `./auto.sh`
- GUI: false
- Formal build: `./tools/build_with_venv.sh` — PASS
- Shell syntax: PASS; `shellcheck` unavailable on host

## Runtime cases

- Case A: PASS. ROS master, Gazebo service/clock/model, 13 controllers, live
  joint feedback, supervisor, `junior_ctrl`, and stable FixedStand reached
  `RUNTIME_ACTIVE`; Ctrl-C left no task ROS/Gazebo process.
- Case B: PASS. `/scan`, `/trunk_imu`, `/scan_pointcloud2`, `/Odometry`, and
  `/cloud_registered` were non-empty/fresh/finite as applicable. The legacy
  converter was disabled.
- Case C startup/state: PASS. DSV/FALCO nodes, planner service, terrain map,
  and navigation relays were ready; confirmed outputs were `FSM=4`, navigation
  enabled, and exploration enabled. The final user instruction accepted a
  simple RTF smoke instead of waiting for a physical exploration trajectory.
- Case D: PASS. With `ENABLE_POINTCLOUD_CONVERTER=1`, the system-Python node
  published `unitree_guide/CustomMsg` on `/livox/lidar2` and
  `sensor_msgs/PointCloud2` on `/livox/Pointcloud2`; observed CPU was about 25%
  versus the invalid baseline venv process at 100% with no ROS output.
- Failure injection: PASS. A temporary launch-only sensor disable with
  `SENSOR_READY_TIMEOUT=5` failed at `STAGE_7_SENSOR_READY`, reported
  `/scan has no publishers`, skipped later modules, and cleaned its process
  tree. The injection edit was reverted immediately.

## Simple RTF smoke

Candidate configuration used `SEED=20260728`, legacy physics
`0.002/500/40`, GUI false, sensors on, FixedStand, converter off, and LiDAR
visualization off. Twenty one-second wall-time samples were:

```text
0.07995, 0, 0.04397, 0.11787, 0, 0.03796, 0, 0.09790, 0, 0.03596,
0.04995, 0.04395, 0, 0.03197, 0.01998, 0.06393, 0, 0.04396, 0, 0.07392
```

- Mean RTF: `0.037064`
- p10/min: `0.0`; max: `0.117871`
- `gzserver`: about 135% CPU
- `junior_ctrl`: about 34% CPU
- `pointcloud2livox`: absent

A fresh old-launcher baseline attempt used the same seed/config except the old
default converter/visualization. It showed `pointcloud2livox` at 100% CPU, but
did not publish `/clock` while another workspace experiment was running, so a
matched numerical improvement claim is not made. After user authorization the
external experiment was stopped, existing files/logs were preserved, and the
standalone candidate smoke above passed. Verdict: simple RTF validation PASS;
quantitative improvement inconclusive.

## Static boundaries

- `gazebo.xacro` diff contains only the two Livox `visualize` values.
- RealSense/depth plugin, topics, rate, resolution, clip range, and enclosing
  sensor condition are unchanged.
- LiDAR ray count, update rate, range, noise, and `/scan` semantics are
  unchanged.
- Remaining sleeps are polling backoff (0.25 s), clock advance sampling
  (0.5 s), short state stability sampling (0.35 s), process termination grace
  (0.1 s), and the final runtime keepalive (1 s).
