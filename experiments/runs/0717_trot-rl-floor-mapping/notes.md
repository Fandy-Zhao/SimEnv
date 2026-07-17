# Experiment Notes

- Date: 2026-07-17
- Branch: `exp/0717-trot-rl-floor-mapping`
- Baseline: `818ee58ee880e12235edf594074787b5de8bb0de`
- Initial worktree status: clean
- Fixed scene: `FLOOR_COUNT=1`, `SEED=77`, spawn `(0.0, 2.3, 0.6, 1.5708)`
- Read before edits: `AGENTS.md`, project-governance `SKILL.md`, `auto.sh`,
  Trotting/RL FSM implementations, FAST-LIO2 launch/README, prior Stage 0/1 records,
  `PROJECT_STATE.md`, and `docs/module_status.md`.
- Planned edits: this experiment bundle, final report, and one status document; control
  source only if runtime evidence demonstrates a safe, reproducible defect.
- Explicitly untouched: main worktree, Stage 2 branch, unrelated modules, external nested repos.

## Runtime log

- Independent worktree build:
  - Torch/CUDA configuration succeeded with explicit system PATH,
    `CMAKE_CUDA_HOST_COMPILER=/usr/bin/g++`, `CMAKE_EXE_LINKER_FLAGS=-no-pie`.
  - Full `catkin_make -j2` is blocked by the unrelated UAV `map_generator`
    target compiling as pre-C++17 while system log4cxx requires `std::shared_mutex`.
  - Required targets built successfully: `junior_ctrl`, `fastlio_mapping`,
    `livox_laser_simulation`, `unitree_servo`, `state_from_gazebo`, Unitree Gazebo
    plugins, and `unitree_legged_control`.
- Trotting attempt 1: invalid and intentionally excluded from metrics. The fresh
  worktree initially lacked `libunitree_legged_control.so`; all twelve joint
  controllers failed to load, so `/Odometry` never published. The bounded trial
  was stopped before movement, its process session was cleaned, and the missing
  target was built before retry.
- Isolation correction: `auto.sh` gained opt-in
  `SKIP_GLOBAL_PROCESS_CLEANUP`; the experiment starts it in a dedicated session
  and kills only PIDs in that session plus its named tmux controller. Default
  startup behavior remains unchanged.
- Python/runtime correction: catkin wrappers generated under Miniconda 3.13
  could leave the scan adapter alive without ROS interfaces. `auto.sh` now runs
  that adapter with `/usr/bin/python3`, and the experiment runner also invokes
  its capture script with `/usr/bin/python3` explicitly.
- Point-cloud export is bounded to 200,000 input points, 50,000 saved points and
  8 seconds. Offline regression: 250,000 synthetic points returned 50,000 saved
  points in 1.03 s initially and 2.54 s in the final system-Python preflight.
- Final fresh epochs (system Python 3.10):
  - Trotting: all 1.75 s ROS-time route segments completed in 30.20 s wall,
    0.192487 m truth displacement, 0.007076 m/s final mean speed, 53 odometry and
    registered-cloud messages, 5,006 saved map points.
  - RL: all route segments completed in 32.11 s wall, 0.169675 m truth
    displacement, 0.024517 m/s final mean speed, 50 odometry and registered-cloud
    messages, 5,048 saved map points.
  - Both truth and odometry datasets contain only finite values. No `/Laser_map`
    frame arrived during the deliberately short trials, so the documented
    fallback uses the last `/cloud_registered` frame (`camera_init`).
  - Map range/outlier audit: Trotting spans x `[-33.069, 2.378]`, y
    `[-4.590, 5.827]`, with 50/5,006 (0.999%) points at x < -10 m. RL spans
    x `[-33.018, 2.363]`, y `[-4.592, 5.836]`, with 56/5,048 (1.109%) points
    at x < -10 m. These remote points are treated as drift/outliers.
