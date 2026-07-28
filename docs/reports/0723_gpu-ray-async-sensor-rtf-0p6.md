# Competition Sensor RTF 0.6 Readiness Report

Date: 2026-07-23

## Verdict

```text
COMPETITION_SENSOR_RTF_0P6_BLOCKED
```

## Governance

- Baseline master HEAD: `5bc0f6fbfdd8333dccbb44c26f216ecfb2811548`
- Task branch: `perf/0723-gpu-ray-async-sensor-rtf-0p6`
- Task worktree: `/home/zzf/search_ws/SimEnv_worktrees/gpu-ray-async-sensor`
- Remote pushed: No
- Skills used: `project-governance`, `cheap-code-worker`

## Scope Audit

- Collision files changed: No
- Collision geometry changed: No
- Physics profile changed: No
- Robot dynamics changed: No
- Controller changed: No

Evidence:

```bash
git diff --name-status 5bc0f6fbfdd8333dccbb44c26f216ecfb2811548...HEAD -- '*collision*' '*.world' '*.urdf' '*.xacro' '*.sdf'
```

The command produced no tracked model, world, xacro, URDF, SDF, or collision-path changes.

## Architecture Findings

- Current LiDAR backend is Gazebo Classic CPU ODE ray, not GPU ray.
- `src/unitree_guide/unitree_ros/robots/a1_description/xacro/gazebo.xacro` instantiates `<sensor type="ray" name="laser_livox">` with `liblivox_laser_simulation.so`, `update_rate=10`, `samples=24000`, `downsample=1`, range `0.1..40`.
- `src/Mid360_imu_sim/src/livox_points_plugin.cpp` creates `LivoxOdeMultiRayShape`, updates rays inside `OnNewLaserScans()`, then constructs and publishes `sensor_msgs/PointCloud` on `/scan`.
- `src/Mid360_imu_sim/src/livox_ode_multiray_shape.cpp` performs `dSpaceCollide2` under the ODE physics update mutex, so ray intersection is CPU physics-collision work.
- PointCloud2 conversion is separate: `src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py` converts `/scan` to `/scan_pointcloud2` with `x/y/z/intensity`.
- FAST-LIO2 consumes `/scan_pointcloud2` and `/trunk_imu` from `src/simenv_fast_lio2_integration/config/simenv_mid360.yaml`.
- Gazebo 11 headers include `GpuRaySensor.hh`, and the host has an NVIDIA RTX 4060 Laptop GPU, but this repository has no `GpuRaySensor` plugin path for the Livox scan-pattern semantics.

The dominant blocking fact is upstream of LiDAR optimization: the formal competition controller baseline without LiDAR already misses the requested 0.60 RTF gate.

## Experiment Matrix

| Candidate | Backend | LiDAR Hz | Async | FAST-LIO2 | Landed RTF | FixedStand RTF | Trotting RTF | Verdict |
| --- | --- | ---: | --- | --- | ---: | ---: | ---: | --- |
| M1 baseline | none | 0 | n/a | off | n/a | 0.405 mean / 0.407 median / 0.390 p10 | not run | FAIL_RTF_BEFORE_LIDAR |
| M2 probe | CPU ODE ray | 10 | no | off | n/a | no `/scan` sample in 8s wall probe; gzserver 163% CPU, RSS 6104688 KiB | not run | SENSOR_RAY_BLOCKED |
| GPU feasibility | not implemented | n/a | n/a | n/a | n/a | blocked by missing semantic-compatible GpuRay plugin | n/a | GPU_NOT_SELECTED_WITH_EVIDENCE |

M1 command:

```bash
SEED=723001 python3 tools/diagnostics/run_core_runtime_matrix.py --out-dir experiments/runs/0723_gpu-ray-async-sensor/baseline_formal --cases M1 M2 M3 M4 --warmup-sim 10 --sample-sim 30 --wall-timeout 900 --startup-clock-timeout 180 --interval 1 --hz-window 5
```

M1 completed before the matrix was stopped at the M2 sensor stall. The M1 sample window excluded startup and had 8 sampled RTF deltas over 30+ sim seconds:

- mean RTF: `0.40537636043045105`
- median RTF: `0.40728587252262166`
- p10 RTF: `0.3899514987690613`
- min RTF: `0.3899514987690613`
- p90 RTF: `0.4134982891005714`
- last sampled process load: `gzserver` CPU `323.5`, `junior_ctrl` CPU `178.6`, `junior_ctrl_threads=15`

M2 evidence during sensor startup:

- `/scan` topic was registered.
- `rostopic hz -w 5 /scan` returned repeated `no new messages`.
- `gzserver` process sample: CPU `163%`, RSS `6104688 KiB`.
- GPU sample during run: utilization `20%`, memory `1138 MiB`; no evidence this came from ray intersection because the active LiDAR code path is ODE CPU ray.

## Selected Configuration

No production configuration is selected. The required RTF target cannot be reached by a LiDAR-only change while the no-LiDAR competition+controller baseline is already below 0.60.

Rollback path: continue using current master behavior. No sensor, collision, physics, controller, or FAST-LIO2 runtime defaults were changed by this task.

## Runtime Equivalence

- Robot control: M1 launched through `auto.sh`, entered controller startup, and was commanded FixedStand by the existing runner.
- LowCmd frequency: not revalidated as PASS in this task; M1 load shows controller active.
- Point cloud direction/frame/timestamp: no new backend selected, so no pointcloud semantic change was introduced.
- FAST-LIO2: not accepted as PASS for this task because full-chain RTF cannot be meaningfully certified after the upstream M1 RTF failure.
- Key obstacles/collision visibility: unchanged because no scene or collision geometry was edited.

## Build and Tests

- Dependency staging: `./tools/external_deps/prepare_fast_lio2_deps.sh --check` and `--prepare` passed.
- Build command: `./tools/build_with_venv.sh`
- Build result: PASS with gcc-11/g++-11, ROS Noetic `/opt/ros/noetic/setup.bash`, worktree `.venv/bin/python`, CUDA root `/usr/local/cuda-11.8`, whitelist `livox_ros_driver;livox_laser_simulation;fast_lio;simenv_fast_lio2_integration;unitree_legged_msgs;unitree_guide;unitree_legged_control;unitree_gazebo`.
- Python syntax: `python3 -m py_compile tools/diagnostics/run_core_runtime_matrix.py tools/diagnostics/check_mapping_pipeline.py src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py` passed.
- Runtime logs: `experiments/runs/0723_gpu-ray-async-sensor/baseline_formal/`.
- Cleanup: task-worktree ROS/Gazebo processes were killed by path-scoped cleanup after the blocked M2 probe.

## Risks and Next Steps

- The existing ODE CPU ray plugin can still be optimized, but it cannot make a no-LiDAR baseline of ~0.405 meet a full-chain 0.60 gate.
- A real GPU LiDAR backend would require a semantic-compatible `GpuRaySensor` implementation or alternate plugin, plus headless rendering validation and pointcloud geometry comparison.
- Before further sensor work, isolate why competition+controller M1 is far lower here than the prior documented M1 `0.989895`, including stale process pollution, CPU contention, and scene/physics/runtime differences.
