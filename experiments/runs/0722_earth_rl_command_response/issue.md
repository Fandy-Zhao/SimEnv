# Issue: Earth RL LowCmd Merge and Command Response

Date: 2026-07-22

## Goal

Safely merge the validated Earth RL LowCmd cadence, Gazebo receive-chain, and
IMU observation fallback fixes into the active RL task branch, then isolate and
minimally fix the near-zero forward motion observed at `vx=0.10 m/s`.

## Scope

- Merge `fix/0722-earth-rl-lowcmd-publisher-stall` into
  `fix/0722-earth-rl-timebase-fast-validation` after auditing its full history.
- Validate build and short Earth RL regression after the merge.
- Create `fix/0722-earth-rl-command-response` in a separate worktree for
  command-to-motion diagnostics and the smallest proven fix.
- Record compact evidence and summaries under
  `experiments/runs/0722_earth_rl_command_response/`.

## Non-Scope

- Do not merge anything into `master`.
- Do not modify `policy_act_inference_stair.pt` or switch policies.
- Do not change physics profiles, friction, reward functions, or broad control
  gains to force motion.
- Do not submit raw large CSV files, rosbags, runtime logs, PID files, generated
  worlds, or build/devel outputs.

## Acceptance Criteria

- Candidate LowCmd/IMU/cadence merge builds with
  `/home/zzf/search_ws/SimEnv/tools/build_with_venv.sh`.
- Short post-merge regression confirms stair policy path/hash,
  `using_imu_policy_input=1`, finite policy input, LowCmd cadence near 500 Hz
  for T0/T1/T2, normal T3/T4 controller cadence, and stable RL zero.
- Command-response branch identifies the first failed stage in the chain from
  `/cmd_vel` through body-frame motion.
- Final `vx=0.10` validation reaches at least
  `RL_COMMAND_RESPONSE_PASS` before merging back.
- Worktree remains clean and master remains untouched.

## Risks

- Gazebo/headless runtime may be unavailable or slow on this host.
- The policy may be genuinely insensitive to low-speed commands after command
  and action semantics are proven correct.
- Existing validated evidence contains small committed diagnostic CSV/JSON
  artifacts from the candidate branch; avoid adding raw large runtime outputs.

## Expected Impacted Modules

- `src/unitree_guide/unitree_guide/`
- `src/unitree_guide/unitree_ros/unitree_legged_control/`
- `experiments/runs/0722_earth_rl_command_response/`
- Status documents such as `PROJECT_STATE.md`, `CHANGELOG.md`, or
  `docs/module_status.md` when meaningful code changes are committed.
