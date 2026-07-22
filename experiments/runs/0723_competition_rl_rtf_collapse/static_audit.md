# Static Audit — Competition RL RTF Collapse

> **⚠ No causal conclusions are drawn from static analysis alone.**
> This document summarises facts found in source code at commit
> `bfbbce24`.  All claims are verifiable against the listed file:line
> references.  Runtime measurement is required to establish causation.

## 1. auto.sh defaults by world mode

### earth (lines 37–49)
| Flag                        | Default |
| --------------------------- | ------- |
| `START_BUILDING_CONTROL`    | 0       |
| `ENABLE_FAST_LIO2`          | 0       |
| `ENABLE_RVIZ`               | 0       |
| `ENABLE_SENSOR_DATA`        | 0       |
| `ENABLE_POINTCLOUD_CONVERTER` | 0    |
| `ENABLE_REFEREE_ODOM`       | 0       |
| `ENABLE_GROUND_TRUTH`       | 0       |
| `WRITE_GENERATED_TRUTH_COPY` | 0      |

### competition (lines 105–123)
| Flag                        | Default |
| --------------------------- | ------- |
| sensor data                 | 1       |
| referee odom                | 1       |
| ground truth                | 1       |
| PointCloud2 converter       | 1       |
| FAST-LIO2                   | 1       |
| RViz                        | 1       |
| building control            | 1       |
| controller (`junior_ctrl`)  | 1       |

Competition mode uses legacy physics defaults (`0.002` / `500` / `40`
ODE iters) **only** when the user has not set `PHYSICS_PROFILE` or any
`GAZEBO_PHYSICS_*` override (lines 141–145).

## 2. GUI / RViz dispatch
- `GUI` is forwarded to `multi_floor_gazeboSim.launch` as `gui:=`
  (line 531).
- `ENABLE_RVIZ` controls the RViz launch (line 648), but RViz is
  **only** started inside the `if [ "$ENABLE_FAST_LIO2" = "true" ]`
  block (line 622).  Setting `ENABLE_RVIZ=1` without
  `ENABLE_FAST_LIO2=1` has no effect.

## 3. RL policy path and loading
- `RL_POLICY_PATH` is exported into the `junior_ctrl` terminal
  environment via `launch_in_terminal` (lines 206–208) and printed in
  the startup summary (lines 507–511).
- The actual Torch model load happens inside
  `State_RL::load_policy()` (`State_RL_test.cpp:871`), which resolves
  the path from ROS param `/rl_policy_path` first, then the env var
  `RL_POLICY_PATH`, then a hard-coded default.

## 4. State_RL lifecycle and inference cadence
- Constructor: calls `load_policy()` (cpp:87).
- `enter()`: sets `infer_thread_runnning = RUNNING`, spawns
  `std::thread(&State_RL::infer_thread_callback, this)` (cpp:151).
- `infer_duration = 0.02` seconds (header:106).
- `run()`: reads from `action_buffer_` — only applies buffered
  actions when the snapshot is valid and the action sequence has
  advanced (cpp:203–223).  No inference is performed in `run()`.

## 5. Torch thread configuration
- A repository-wide `rg` search for `set_num_threads`,
  `OMP_NUM_THREADS`, `MKL_NUM_THREADS`, `TORCH_.*THREAD`,
  `at::set_num_threads`, `intra_op_parallelism`, and
  `inter_op_parallelism` returned **no matches**.
- The inference thread and FAST-LIO2 threads therefore run with
  default Torch / OpenBLAS / MKL thread pool sizes, which may
  oversubscribe CPU cores when both are active.

## 6. Mapping topic chain
```
/scan (sensor_msgs/PointCloud, from Gazebo)
  │ scan_to_pointcloud2.py
  ▼
/scan_pointcloud2 (sensor_msgs/PointCloud2)
  │ FAST-LIO2 (fastlio_mapping / laserMapping)
  ▼
/Odometry          (nav_msgs/Odometry)
/cloud_registered  (sensor_msgs/PointCloud2)
  │ relay topics exposed by simenv_fast_lio2_mapping.launch
  ▼
/state_estimation   (nav_msgs/Odometry)
/registered_scan    (sensor_msgs/PointCloud2)
```
`scan_to_pointcloud2.py` is started at line 634;
`simenv_fast_lio2_mapping.launch` is launched at line 640.

## 7. Static Hypotheses To Test
- If M1 is already far below M0, competition scene/contact complexity is a
  primary suspect before LiDAR, FAST-LIO2, or RL inference.
- If M2 drops sharply from M1, Gazebo LiDAR plugin cost is isolated.
- If M3 drops from M2 or `/scan_pointcloud2` lags, the converter is a suspect.
- If M4 drops from M3 or outputs stop, FAST-LIO2 cost or timing is a suspect.
- If M5 drops from M4 without entering RL, model load or hidden inference must
  be checked against controller logs.
- If M6 drops with mapping disabled, RL inference/control or Torch threading is
  a suspect.
- If M7 drops beyond M4 and M6, combined CPU/thread contention is a suspect.
- If M8 differs mainly from M7, GUI/gzclient/RViz overhead is implicated.
