# Earth RL Fastcheck Summary

## Task Result

- Total verdict: `EARTH_RL_RTF_BLOCKED`
- Fix applied: Yes. The RL default policy path was changed from `policy_act_inference_plane.pt` to `policy_act_inference_stair.pt`.
- Core conclusion: the controller now loads the requested stair policy, but Earth `normal` RTF is not stable enough by the requested gate and the first RL `vx=0.10 m/s` smoke test falls.

## Governance

- Baseline master HEAD: `38947a556342b4bafa20a84ff56e8065ce4f358f`
- Task branch: `fix/0722-earth-rl-timebase-fast-validation`
- Task worktree: `/home/zzf/search_ws/SimEnv_worktrees/earth-rl-fastcheck`
- Final HEAD: pending commit at report time
- Original master workspace clean after worktree creation: Yes
- Merged to master: No

## Runtime Configuration

- `WORLD_MODE`: `earth`
- `PHYSICS_PROFILE`: `normal`
- `GUI`: `False`
- Effective world: `generated_building/earth_physics.world`
- Physics parameters: `max_step_size=0.001`, `real_time_update_rate=1000`, `ode_iters=20`, `contact_max_correcting_vel=5.0`
- `/use_sim_time`: `true`
- `/clock`: published and advanced
- Startup command: `WORLD_MODE=earth PHYSICS_PROFILE=normal GUI=False ./auto.sh`

## RTF Result

| mean | median | p10 | minimum | duration | verdict |
|---:|---:|---:|---:|---:|---|
| 0.679852 | 0.750945 | 0.407949 | 0.293591 | 25 s wall | `RTF_FAIL` |

The median exceeds `0.60`, but p10 is below `0.55`, so the requested stable-RTF gate fails.

## Policy Result

| configured path | runtime resolved path | SHA256 | verdict |
|---|---|---|---|
| `src/unitree_guide/logs/policy_act_inference_stair.pt` | worktree-relative path printed by `junior_ctrl`; content matches `/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_stair.pt` | `2d5aa72511c0c6609c02f4105845eee6974d3d73431497f8f35306da9588fe14` | `POLICY_PATH_PASS` |

## Timebase Result

| robot state Hz | observation Hz | inference Hz | command Hz | target Hz | simulation-time period | wall-time period | jitter | duplicate-state count | history duplicate ratio | deadline misses | RTF/control-period correlation | verdict |
|---:|---:|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---|
| 643.60 FSM rows / 414.34 accepted LowCmd Hz | 48.08 policy-wait Hz | 48.10 action Hz | 414.34 LowCmd Hz | 500 FSM, 50 policy | median 2000 us, mean 2413 us | median 2.070 ms | 2.080 ms stdev | 11970 | 0.002309 | sum 15252, max 21 | r=0.267 | `TIMEBASE_FAIL` |

RTF does change the effective command publication density: the policy loop remains near 50 Hz in simulation time, but accepted low-level command publication is about 414 Hz instead of the 500 Hz target and has deadline misses. History duplication is low, but not zero.

## RL Speed Results

| cmd_vx | measured_vx | tracking_gain | MAE | vy | yaw_rate | lateral_drift | yaw_drift | RTF | stable | verdict |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 0.10 | 0.003374 median | 0.033738 | 0.123207 | 0.005812 | -0.012121 | -0.262036 | 0.309129 | 1.015 median in probe window | false | `UNSTABLE` |

The run stopped after `0.10 m/s` per the task rule because the robot fell or lost stable base height (`min_base_height=0.078849 m`).

## Validated Operating Range

- Lowest effective speed: none validated
- Recommended speed range: none
- Highest verified stable speed: none
- First failed speed: `0.10 m/s`
- Failure mode: unstable RL smoke; near-zero forward median velocity followed by base-height fall.

## Root Cause

- Direct evidence: runtime `junior_ctrl` initially would have loaded the source default `policy_act_inference_plane.pt`; this task changed it to the requested stair policy and verified the printed loader path plus SHA256.
- Direct evidence after fix: RL entered from FixedStand (`/fsm/state_cmd` raw `6`, mapped RL enum `8`), ACTION and LOWCMD timing rows were produced, and `/cmd_vel` smoke data was captured.
- Root cause for policy gate: wrong default model path in `State_RL::load_policy()`.
- Root cause for overall blocked verdict: Earth `normal` RTF did not satisfy the requested p10 stability gate, and the first active RL smoke was unstable even after policy-path correction.
- Relation to RTF: yes for the task verdict and command-period quality; policy inference stayed near 50 Hz sim-time, but 500 Hz command publication did not hold.
- Relation to policy path: policy path was a confirmed defect and was fixed; motion still failed after loading the requested stair policy.

## Changes

- `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_RL_test.cpp`: changed the default RL policy from plane to stair.
- `experiments/runs/0722_earth_rl_fastcheck/scripts/analyze_timing_csv.py`: added a small TimingDiagnostics CSV analyzer.
- `experiments/runs/0722_earth_rl_fastcheck/scripts/earth_rl_speed_probe.py`: added a short ROS/Gazebo speed probe for FixedStand-to-RL smoke and bounded speed sweeps.
- `experiments/runs/0722_earth_rl_fastcheck/issue.md`: recorded task scope, non-scope, acceptance, and risk.
- `experiments/runs/0722_earth_rl_fastcheck/*/*.json`: small metric summaries from this run.

## Validation Commands

- `git branch --show-current`
- `git rev-parse HEAD`
- `git status --porcelain`
- `git worktree list`
- `git worktree add -b fix/0722-earth-rl-timebase-fast-validation /home/zzf/search_ws/SimEnv_worktrees/earth-rl-fastcheck master`
- `rg -n "State_RL|infer_thread|policy_act_inference|ros::Time|WallTime|getTime|clock_gettime|sleep|usleep|absoluteWait|Rate|Timer" src/unitree_guide`
- `python3 -m py_compile experiments/runs/0722_earth_rl_fastcheck/scripts/analyze_timing_csv.py experiments/runs/0722_earth_rl_fastcheck/scripts/earth_rl_speed_probe.py`
- `/home/zzf/search_ws/SimEnv_worktrees/earth-rl-fastcheck/tools/build_with_venv.sh`
- `WORLD_MODE=earth PHYSICS_PROFILE=normal GUI=False ./auto.sh`
- `rostopic echo -p /gazebo/performance_metrics`
- `python3 experiments/runs/0722_earth_rl_fastcheck/scripts/analyze_timing_csv.py ...`
- `/usr/bin/python3 experiments/runs/0722_earth_rl_fastcheck/scripts/earth_rl_speed_probe.py --speeds 0.10,0.20,0.30,0.40,0.50`

## Commits

- Pending at report time.

## Final Recommendation

先解决 RTF 再测试。
