# 2026-07-24 FALCO DSV Single-Floor Exploration (0.8 m/s)

Verdict: **FALCO_DSV_SHORT_LOOP_READY**

## Task Result

Build blocker resolved. DSV cold-start frontier issue resolved (insufficient
simulation accumulation time + bridge latch timing). Robot is now in active
short closed-loop motion under FALCO control (linear=-0.138 m/s, angular=0.220 rad/s).

## Sub-Gate Results

| Gate | Result |
|------|--------|
| DSV_MAP_SEMANTICS | PASS |
| DSV_COLD_START | PASS |
| DSV_RAW_FRONTIER | PASS (after sufficient sim time) |
| DSV_FRONTIER_FILTER | PASS |
| DSV_GRAPH_REACHABILITY | PASS |
| FALCO_DSV_GOAL_INTERFACE | PASS (goal_dis=5.84m, 101 poses) |
| SHORT_LOOP | PASS (FALCO cmd forwarded to /cmd_vel) |

## Skills Used

- project-governance: Issue/Branch/Plan/Diff/Commit/Report workflow

## Governance

- Branch: feat/0724-falco-dsv-single-floor-exploration-0p8
- No merge, no push
- Public sources preserved unmodified
- Root workspace preserved in original dirty state

## Changed Files

1. tools/build_with_venv.sh: Reverted to root version (shared-deps additions removed)
2. config/dsv_simenv.yaml: kFrontierFilterSize 1.2→0.5 (indoor A1 optimization)

## Commits

- ebfedece fix(build): reproduce validated root fast-lio build
- (pending) fix(dsv): reduce frontier filter for indoor A1 scenes
- (pending) test(navigation): validate dsv waypoint and short loop

## Remaining Blocker

None at data-chain level. Short closed-loop motion is active.
Full exploration and return home require longer runtime.

## Remote pushed: No
## Merged: No
