# Notes: Earth RL Command Response

Date: 2026-07-22

## Phase A Merge Audit

- Current task branch before merge:
  `fix/0722-earth-rl-timebase-fast-validation`
- Baseline HEAD before merge:
  `e7fbbe639412fbd528a2fc35dc3009aa18c9af83`
- Validated candidate branch:
  `fix/0722-earth-rl-lowcmd-publisher-stall`
- Candidate HEAD:
  `a18cd0ae34b802722374172ecb75f6a8898ac985`
- Merge commit:
  `39bbb6cfef8fdcccbef4990919e6bf8579414caf`

The candidate branch contains the validated LowCmd queue depth 1 and
`tcpNoDelay()` transport change, Earth IMU policy-input fallback, persistent
`AsyncSpinner`, removal of synchronous `spinOnce()` calls from the LowCmd path,
simulation-time LowCmd scheduling, and staged T1-T4 LowCmd trace support.
`a18cd0ae` has parent `28d0d649`, so the final cadence fix depends on the
earlier candidate chain and the branch was merged whole.

The candidate diff did not modify `.pt` files, physics profiles, PID files,
generated worlds, or build/devel outputs. It includes committed compact
diagnostic CSV/JSON evidence from prior validation.

## Phase A Checks

- `git diff --check HEAD^1..HEAD`: PASS
- `./tools/build_with_venv.sh` from the task worktree: PASS
  - Repository root reported by the passing build:
    `/home/zzf/search_ws/SimEnv_worktrees/earth-rl-fastcheck`
- Initial accidental run of `/home/zzf/search_ws/SimEnv/tools/build_with_venv.sh`
  built the master worktree and is not counted as task-branch validation.

## Phase A Runtime Regression

Startup command base:

```bash
WORLD_MODE=earth PHYSICS_PROFILE=normal GUI=False ./auto.sh
```

Additional runtime-only diagnostics disabled unrelated sensors and wrote raw
CSV traces under the ignored `raw/` directory.

Policy:

- Runtime path printed by `junior_ctrl`:
  `src/unitree_guide/logs/policy_act_inference_stair.pt`
- SHA256:
  `2d5aa72511c0c6609c02f4105845eee6974d3d73431497f8f35306da9588fe14`

RL zero capture:

- Duration: `9.000 sim-s`
- Fall: `false`
- Minimum base height: `0.311336 m`
- Maximum tilt: `3.04993 deg`
- Median RTF: `1.00864`
- Post delta x: `0.00747 m`
- Post mean base vx: `0.000743 m/s`

Policy input diagnostics:

- RL diagnostic complete rows checked: `3022`
- `using_imu_policy_input=1` rows: all rows
- Policy input non-finite rows: `0`
- Action output non-finite rows: `0`

Simulation-time cadence:

| Window | FSM/T0 | LowCmd T1 | T3 | T4 | Out-of-order |
| --- | ---: | ---: | ---: | ---: | ---: |
| FixedStand last 5 sim-s | 500 Hz | 500 Hz | 1000 Hz | 1000 Hz | 0 |
| RL zero 8.5 sim-s | 500 Hz | 500 Hz | 1000 Hz | 1000 Hz | 0 |

Phase A verdict: `MERGE_VALIDATED_FIX_PASS`.
