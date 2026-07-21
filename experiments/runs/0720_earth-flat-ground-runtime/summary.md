# Runtime Summary

## Verdict

`TASK_PARTIAL`

The flat Earth world loads and the deleted platform models are absent at runtime, but the validation gate stops at G1 and C0:

- G0 world-only: `G0_PASS`
- G1 initial contact/controller epoch: `G1_FAIL_ATTITUDE`
- C0 competition FixedStand rerun: `FAIL_ATTITUDE`
- Earth E0 FixedStand: `BLOCKED_BY_G1`
- RL E1/E2/E3/E5: `BLOCKED_BY_E0`

## Key Evidence

| Case | Verdict | RTF median | Max tilt deg | Min base height m | Forward m | Lateral m | Yaw deg |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| G0_world_flat | G0_PASS | 0.983342 | n/a | n/a | n/a | n/a | n/a |
| G1_spawn_contact_controller | G1_FAIL_ATTITUDE | 0.988074 | 5.188578 | 0.050442 | 0.008432 | 0.000271 | 0.020319 |
| C0_competition_fixedstand_rerun | FAIL_ATTITUDE | 0.587271 | 170.681916 | 0.078872 | -0.276650 | -0.005954 | 111.200178 |

## Platform Runtime Evidence

G0 `/gazebo/model_states` names:

```text
ground_plane
```

G1 `/gazebo/model_states` names:

```text
a1_gazebo
ground_plane
```

`platform_1` and `platform_2` are absent in both runtime checks.

## Root Cause Closure

Not closed. The platform removal is verified at source and runtime, but the required closure chain is incomplete:

- Spawn pose unchanged: yes.
- Platforms removed: yes.
- C0 competition no-regression: no, C0 rerun failed.
- New Earth E0 3/3 pass: not run, blocked by G1.

Conclusion: `PLATFORM_CONTACT_CONFIRMED_CONTRIBUTOR_BUT_NOT_SOLE_CAUSE` is not proven by this run; the safer verdict is that platform removal is integrated, but runtime validation is blocked by initial-contact/entry-chain failure in the selected artifact environment.

## User-Requested Stair Policy Follow-Up

The stair policy exists at `/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_stair.pt` and has sha256 `2d5aa72511c0c6609c02f4105845eee6974d3d73431497f8f35306da9588fe14`. It was not switched in this run because RL is gated behind E0, and E0 was blocked.
