# Earth World A1 RTF Diagnosis

## Executive Summary

In the isolated diagnosis worktree, the low Gazebo real-time factor (RTF) in `earth.world` is not caused by FAST-LIO2, RViz, lidar, PointCloud2 conversion, referee odometry, building generation, or the multi-floor competition scene. Those components were disabled or not launched.

The strongest measured cause is the combination of A1 ground contact with the `earth.world` physics configuration:

- `earth.world` uses ODE quick solver at `max_step_size=0.0002`, `real_time_update_rate=5000`, `iters=50`, `sor=1.3`.
- Empty `earth.world` ran near real time: average RTF `0.9975`.
- A1 suspended with the same model and disabled sensors ran near real time: average RTF `0.9607`.
- A1 landed with no `junior_ctrl` process dropped to average RTF `0.1370`.
- The same landed case with a runtime-only physics override to `max_step_size=0.004`, `real_time_update_rate=250`, `iters=20` recovered to average RTF `0.9999`.
- FixedStand, with `junior_ctrl` running at 500 Hz, measured average RTF `0.8912`, showing controller CPU can compete for resources but is not required to reproduce the severe low-RTF condition.

Final verdict: the observed RTF near `0.1` is primarily an ODE contact/friction solve cost amplified by a very small 0.2 ms physics step and 50 solver iterations. Controller timing warnings such as `absoluteWait(2000)` are better interpreted as a symptom of wall-time overload and time-source mismatch risk, not as the root cause of Gazebo RTF collapse.

## Scope and Non-Goals

This task only diagnosed `earth.world` with Unitree A1. It did not run competition building generation, multi-floor randomized scenes, hazard spawning, mapping, FAST-LIO2, RViz, lidar, PointCloud2 conversion, referee odometry, or building control.

No source, launch, world, URDF/Xacro, controller, script, configuration, or physics file was modified to improve RTF. The only repository change is this Markdown report.

## Branch and Worktree Isolation

- Original workspace: `/home/zzf/search_ws/SimEnv`
- Original branch: `backup/0720-root-uncommitted-state`
- Original HEAD: `f489e553e06c680069c75cfbc3fdccaed184edad`
- Diagnosis worktree: `/home/zzf/search_ws/SimEnv-earth-rtf-diagnosis`
- Diagnosis branch: `diagnose/earth-world-a1-rtf`
- Task baseline local `master`: `daa859f866206fab65f42525ea7633054b4b8bd9`
- Baseline commit title: `Merge branch 'fix/0721-unitree-runtime-rebuild-and-retest' into master`

`git fetch --all --prune` updated `origin/master` to `9111f203`, but the `github` remote failed with SSH public-key denial. Local `master` and `origin/master` were divergent (`master...origin/master`: local ahead 133, remote ahead 5). To avoid importing unrelated remote changes or rewriting local `master`, the task branch was created from the local clean `master` worktree.

## Original Workspace Preservation

The original workspace was dirty before the task, including modified `logs/competition_gazebo.log`, `logs/competition_gazebo.pid`, many untracked experiment outputs, and untracked source/package directories. The task did not switch, reset, stash, clean, or edit that workspace.

## Test Environment

- ROS: Noetic
- Gazebo: Gazebo Classic via `gazebo_ros`
- Worktree build: local `catkin_make` in the diagnosis worktree
- Build note: full build failed with Torch/CUDA enabled because nvcc could not execute `cc1plus`; a runtime-only diagnostic build succeeded with `UNITREE_ENABLE_TORCH_POLICY=OFF` and a catkin whitelist for earth/A1 packages.
- Python note: direct ROS launch initially failed under inherited Conda Python; clean runs unset `PYTHONHOME` and `PYTHONPATH`, set `PYTHONNOUSERSITE=1`, and used system paths, matching `auto.sh` intent.
- Perf note: `/proc/sys/kernel/perf_event_paranoid` was `4`; `perf` profiling was not run because changing system permissions was out of scope.

## Earth World Launch Configuration

`WORLD_MODE=earth` is implemented in `auto.sh`. In earth mode it sets:

- `BUILDING_WORLD_FILE` to `src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world`
- `START_BUILDING_CONTROL=0`
- `ENABLE_FAST_LIO2=0`
- `ENABLE_RVIZ=0`
- `ENABLE_SENSOR_DATA=0`
- `ENABLE_POINTCLOUD_CONVERTER=0`
- `ENABLE_REFEREE_ODOM=0`
- `ENABLE_GROUND_TRUTH=0`
- `WRITE_GENERATED_TRUTH_COPY=0`

The launch entry is `src/unitree_guide/unitree_guide/unitree_guide/launch/multi_floor_gazeboSim.launch`. It still loads A1, `gazebo_ros_control`, 12 joint controllers, robot state publisher, the trunk IMU, and four foot contact sensors unless A1 itself is skipped by using a separate empty-world launch.

## Physics Configuration

`earth.world` contains:

| Parameter | Value |
| - | - |
| Physics engine | ODE |
| `max_step_size` | `0.0002` s |
| `real_time_update_rate` | `5000` Hz |
| Solver type | `quick` |
| ODE `iters` | `50` |
| ODE `sor` | `1.3` |
| `cfm` | `0.0` |
| `erp` | `0.2` |
| `contact_max_correcting_vel` | `10.0` |
| `contact_surface_layer` | `0.001` |

This is substantially heavier than a 2 ms / 500 Hz / ODE 40 setup.

## Disabled Components

During measured runs:

- FAST-LIO2: disabled
- RViz: disabled
- A1 lidar/depth sensor block: disabled with `enable_sensor_data:=false`
- PointCloud2 conversion: disabled
- Referee odometry: disabled
- Ground truth P3D plugins: disabled
- Building generation/control: not used
- Competition scene: not run

## Measurement Methodology

Runs used independent ROS/Gazebo ports and short raw logs under `/tmp`. Stable windows were collected after startup delay using:

- `gz stats -p -d 15` or `-d 20`
- `ps` for process CPU/RSS
- `top -H` for gzserver thread snapshots
- `rostopic hz -w 80 /clock`
- `rostopic hz -w 80 /a1_gazebo/joint_states`

Some early runs were discarded because inherited Conda Python broke xacro, or ROS `run_id` conflicts prevented Gazebo startup. The valid measurements below come from clean-environment runs.

## Experiment Matrix

| Case | Description | Runtime changes |
| - | - | - |
| Empty earth | `earth.world` only, no A1 | none |
| Suspended A1 | A1 spawned with `user_debug:=True`, sensors off | existing Xacro debug arg |
| Landed A1, no junior controller | A1 landed, ros_control loaded, no `junior_ctrl` | none |
| FixedStand | Landed A1 plus `junior_ctrl`, `/fsm/state_cmd=2` | none |
| Landed A1, physics override | Landed A1, no `junior_ctrl` | runtime `gz physics -s 0.004 -u 250 -i 20` only |

## Baseline Results

| Scenario | Average RTF | Min RTF | Max RTF | gzserver CPU | Notes |
| - | -: | -: | -: | -: | - |
| Empty `earth.world`, no A1 | `0.9975` | `0.9600` | `1.0000` | `56.5%` | world alone is not the bottleneck |
| A1 suspended, sensors off | `0.9607` | `0.9400` | `0.9700` | `152%` | model/plugins without foot-ground contact remain near real time |
| A1 landed, no `junior_ctrl` | `0.1370` | `0.1100` | `0.1500` | `236%` | severe RTF collapse reproduced without active controller |
| FixedStand | `0.8912` | `0.8500` | `0.9300` | `214%` | controller adds CPU (`junior_ctrl` `60.2%`) but severe collapse was not present in this stable FixedStand sample |
| Landed, runtime 4 ms / 250 Hz / ODE 20 | `0.9999` | `0.9900` | `1.0000` | `73.0%` | temporary diagnostic override restores real-time behavior |

## Contact Isolation Results

The contact isolation result is the strongest evidence:

- A1 suspended: average RTF `0.9607`
- A1 landed, no `junior_ctrl`: average RTF `0.1370`

Because both cases used the same world, same A1 model, disabled sensors, disabled FAST-LIO2, disabled RViz, and no active `junior_ctrl`, the large drop is attributable to ground contact, collision/contact constraints, and ODE solve cost.

## Controller-State Comparison

FixedStand ran at average RTF `0.8912`, while landed without `junior_ctrl` measured `0.1370`. This single clean run does not prove FixedStand is always cheaper; it does prove that `junior_ctrl`/RL inference is not necessary to reproduce severe low RTF.

Trotting and RL were not runtime-measured in this task because the clean isolated build had to disable Torch policy to avoid the local CUDA compiler failure. Static review shows Trotting/RL code is compiled out under `UNITREE_ENABLE_TORCH_POLICY=OFF`, and prior committed project reports should be used only as secondary context, not as new evidence for this branch.

## Physics Parameter Sensitivity

The landed no-controller case recovered from average RTF `0.1370` to `0.9999` when physics was changed at runtime to:

- `max_step_size=0.004`
- `real_time_update_rate=250`
- `iters=20`

No file was edited. This sensitivity confirms the primary bottleneck is physics-step/contact-solver cost, not a standalone sensor or mapping process.

## CPU and Thread Profiling

`ps` and `top -H` snapshots showed:

- Landed no-controller: `gzserver` about `236%` CPU, RSS about `472084 KB`
- Suspended A1: `gzserver` about `152%` CPU, RSS about `473200 KB`
- FixedStand: `gzserver` about `214%` CPU, `junior_ctrl` about `60.2%` CPU
- Runtime 4 ms / 250 Hz / ODE20: `gzserver` about `73.0%` CPU

Thread snapshots showed multiple active `gzserver` threads rather than an obviously idle process. Symbol-level `perf` hotspots were not collected because `perf_event_paranoid=4` and changing kernel permissions was out of scope.

## Collision Model Review

A1 collision geometry is mostly primitive:

- trunk: box
- hip: cylinder
- thigh: box
- calf: box
- foot: sphere radius `0.02`
- IMU helper links: tiny boxes

However:

- `self_collide` is enabled on thigh, calf, and foot links.
- foot links use friction `mu1=0.6`, `mu2=0.6`.
- several links specify Gazebo contact stiffness `kp=1000000.0`, `kd=1.0`.
- the sensor carrier links exist even when sensor data plugins are disabled; `laser_livox` and `real_sense` have mesh collisions in the Xacro, although they are not the direct foot-ground contact cause.

The foot collision primitive is not a high-poly mesh, so mesh complexity is not the top explanation for the 0.1 RTF. Self-collision and stiff contact parameters can still amplify ODE constraint work.

## Gazebo Plugin Review

Gazebo-side A1 plugins and runtime components include:

- `libgazebo_ros_control.so`
- `libgazebo_ros_force.so`
- trunk `libgazebo_ros_imu_sensor.so` at 1000 Hz
- four contact sensors at 100 Hz with `libunitreeFootContactPlugin.so`
- 12 `unitree_legged_control/UnitreeJointController` controllers
- `joint_state_controller` at 1000 Hz

The foot contact plugin publishes a wrench message on each contact sensor update. This is secondary compared with ODE contact solving, but it remains in `gzserver` and contributes to load after sensors such as lidar are disabled.

## Simulation-Time vs Wall-Time Analysis

The launch uses `/use_sim_time=true` and Gazebo publishes `/clock`.

The controller stack mixes sources:

- FSM/control timing uses `getSystemTime()` / `gettimeofday()` for `absoluteWait`.
- FSM records and scheduling also consult `ros::Time::now()`.
- IOROS uses `/clock` to stamp accepted state time.
- RL policy code, when built, uses both `getTime()` and `ros::WallTime`.

At low RTF, a 2 ms wall-time `absoluteWait` loop can run faster relative to simulation time than the physics world advances. This can produce repeated-state sampling, history-buffer duplication risk, and warning spam. It is mainly a consequence of slow simulation and mixed time semantics, not the primary Gazebo physics bottleneck.

## absoluteWait Warning Interpretation

`absoluteWait(2000)` warnings mean one controller iteration exceeded its 2 ms wall-time budget. In a world already slowed by ODE contact solving, such warnings are expected. They can worsen CPU contention and controller synchronization, but the landed no-controller experiment proves they are not required for the severe RTF drop.

## Root-Cause Ranking

| Rank | Candidate cause | Evidence | Confidence |
| -: | - | - | - |
| 1 | ODE foot-ground contact and friction solving | suspended A1 `0.9607` vs landed no-controller `0.1370` | High |
| 2 | 0.2 ms / 5000 Hz / ODE 50 physics configuration | runtime 4 ms / 250 Hz / ODE20 restored RTF to `0.9999` | High |
| 3 | self-collision and stiff contact parameters | self-collide enabled on leg links; `kp=1000000`; likely increases constraints | Medium |
| 4 | Gazebo-side ros_control, 1 kHz IMU/joint state, contact plugin publishing | present with sensors disabled; CPU load in `gzserver` | Medium |
| 5 | `junior_ctrl` wall-time loop and controller CPU | FixedStand controller CPU `60.2%`; severe low RTF reproduced without it | Low to Medium |
| 6 | RL/Trotting inference | not runtime-verified here; Torch build unavailable; not needed for no-controller low RTF | Low |
| 7 | lidar / FAST-LIO2 / RViz / PointCloud2 | disabled or not launched in low-RTF reproduction | Excluded for this case |

## Verified Findings

1. Empty `earth.world` is not the main cause of RTF `0.1`.
2. Loading A1 suspended keeps RTF near real time.
3. A1 ground contact alone can drop RTF to about `0.14`.
4. Runtime physics relaxation restores landed A1 to near real time.
5. FAST-LIO2, RViz, lidar, PointCloud2 conversion, ground truth, referee odometry, and building control were not required for the low-RTF reproduction.
6. `earth.world` actually uses 0.2 ms / 5000 Hz / ODE 50, not 2 ms / 500 Hz / ODE 40.

## Unverified Hypotheses

- Whether Trotting is consistently lower than FixedStand in this exact branch could not be measured because Torch policy could not be built in the isolated worktree.
- Whether RL inference is a meaningful CPU competitor in this exact branch remains unverified at runtime.
- Symbol-level `dWorldQuickStep`, `dxQuickStepper`, or `dSolveLCP` hotspot attribution was not collected because `perf` permissions were restricted.
- The exact contribution of self-collision versus ground contact cannot be separated without a no-file-change runtime switch or a separate model variant, which was out of scope.

## Recommended Follow-Up Optimizations

These are recommendations only; none were implemented in this task.

1. Reconsider `earth.world` physics defaults for A1: start with 1-2 ms step and lower ODE iterations, then validate controller stability.
2. Add an explicit launch/runtime profile for diagnosis versus high-fidelity physics so heavy settings are deliberate.
3. Test self-collision disabled on noncritical leg links in a separate optimization branch.
4. Review contact stiffness/friction values for feet and contact surface stability.
5. Move controller loops further toward simulation-time gating and repeated-state rejection under low RTF.
6. Add a lightweight RTF regression harness that records `/clock`, `gz stats`, controller CPU, and process metadata without committing raw logs.

## Repository Change Audit

Intended repository change:

- `docs/diagnostics/earth_world_a1_rtf_diagnosis.md`

Not changed:

- business code
- controller code
- URDF/Xacro
- world files
- launch files
- scripts
- configuration files
- physics parameters in repository files

Build artifacts and raw logs were local/untracked and are not part of the final commit.

## Final Verdict

The low RTF around `0.1` in `earth.world` with Unitree A1 is a physics/contact bottleneck: A1 foot-ground contact under `earth.world`'s 0.2 ms / 5000 Hz / ODE 50 configuration overwhelms Gazebo. Disabling lidar, FAST-LIO2, RViz, PointCloud2 conversion, and competition components is insufficient because those components are not the dominant cost in this scenario.
