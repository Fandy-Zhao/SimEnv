# Repository Consolidation Follow-up: Earth RL Motion

## Verdict

`EARTH_INTEGRATION_PASS_RL_FAIL` + `TASK_PARTIAL`

Earth mode is integrated and can launch `earth.world` independently. RL motion
testing is blocked before policy evaluation because E0 FixedStand enters state
2 but holds an invalid body attitude on the new terrain.

## Branch

- Branch: `test/0720-earth-rl-motion`
- Worktree: `/home/zzf/search_ws/SimEnv_worktrees/earth-rl-motion`
- Base: `84ff02d7022c11dbb26fdbb2ff37322bf4aaf814`

## Scope

Changed `auto.sh` world selection, added the tracked `earth.world`, and added
measurement-only benchmark tooling. No control behavior, FSM state logic, RL
model, observation order, action scale, gait, IK, estimator, or validator logic
changed.

## Earth Integration

| Item | Result |
| --- | --- |
| Default competition preserved | PASS |
| Earth world resolved | PASS |
| Competition generation skipped | PASS |
| Optional nodes isolated | PASS |
| Absolute paths absent from code | PASS |
| Static tests | PASS |

## Runtime Matrix

| Test | Command | Duration | RTF | Displacement | Drift | Fall | FSM | Verdict |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| E0 FixedStand | none | 15.174 s | not aggregated | +0.019650 m | 0.108354 m | no height fall | 2 | FAIL_ATTITUDE |
| E1 RL zero | vx=0 | not run | n/a | n/a | n/a | n/a | n/a | BLOCKED_BY_E0 |
| E2 RL 0.05 | vx=0.05 | not run | n/a | n/a | n/a | n/a | n/a | BLOCKED_BY_E0 |
| E3 RL 0.10 | vx=0.10 | not run | n/a | n/a | n/a | n/a | n/a | BLOCKED_BY_E0 |
| E4 RL 0.20 | vx=0.20 | not run | n/a | n/a | n/a | n/a | n/a | BLOCKED_BY_E0 |
| E5 Stop | vx=0.10 then 0 | not run | n/a | n/a | n/a | n/a | n/a | BLOCKED_BY_E0 |
| E6 Turn | yaw left/right | not run | n/a | n/a | n/a | n/a | n/a | BLOCKED_BY_E0 |
| E7 Terrain | region-specific | not run | n/a | n/a | n/a | n/a | n/a | BLOCKED_BY_E0 |

## Diagnostic Evidence

- Gate V / Gate P: existing diagnostic semantics retained; this task did not
  alter validators or pre-wave gates.
- Timing alignment: controller timing CSV was produced in E0; final FSM state
  was 2.
- Fast exit / shared-base failure: not triggered as an RL fast exit because RL
  was not entered.
- Model loading: Torch-enabled build failed before active RL due CUDA compiler
  probe (`nvcc` cannot execute `cc1plus`).
- Command publication: E0 used `/fsm/state_cmd` to enter FixedStand.
- `/clock`: present in earth topic smoke.
- Gazebo model states: present; models included `ground_plane`, `platform_1`,
  `platform_2`, and `a1_gazebo`.

## Tests

| Test | Result |
| --- | --- |
| Python unit tests | PASS, 7 tests |
| Python compile | PASS |
| Shell syntax | PASS |
| C++ build | FAIL, environment/dependency blockers |
| Existing timing tests | Not rerun; control code unchanged |
| Earth integration tests | PASS, static checks |
| Gazebo smoke | PASS for world/topic startup |
| RL runtime | BLOCKED by E0 FixedStand attitude failure |

## Root Cause

Current evidence points first to `world integration / spawn/contact`, not to the
RL policy. FixedStand entered successfully, but model attitude reached about
`91.91 deg` roll while height stayed near `0.495 m`. Active RL diagnosis should
wait until the earth spawn/contact pose is stable.

## Control Behavior Changes

None.

## Recommended Next Step

Adjust and validate the earth spawn/contact pose on `earth.world` before any RL
recovery work.
