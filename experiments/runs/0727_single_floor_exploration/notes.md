# Experiment notes

- Baseline local master: `48d7b8126ab5e98784f999f058167f5a083ca9fa`
- Task branch: `test/0727-single-floor-exploration-artifacts`
- Task worktree: `/home/zzf/search_ws/SimEnv_worktrees/single-floor-exploration-artifacts-0727`
- Root dirty-state backup: `/home/zzf/search_ws/SimEnv_backups/0727_single_floor_pre_task/`
- Formal build entry: `/home/zzf/search_ws/SimEnv/tools/build_with_venv.sh`
- Formal runtime entry: `WORLD_MODE=competition GUI=False ./auto.sh`

## Observed result

- Required build wrapper passed after each source repair; final log:
  `build_after_angular_step_threshold_fix.log`.
- Final attempt ran through `./auto.sh` in a one-floor competition scene with
  seed 727. Supervisor outputs held `enabled=true`, `fsm=4`, and
  `exploring=true`; `/cmd_vel` had only `cmd_vel_bridge` as publisher.
- FAST-LIO, map projection, DSV, waypoint, FALCO path, velocity bridge, and
  Unitree Trotting all produced live evidence. One unique DSV goal was saved.
- Recorder saved 97 trajectory samples, 0.24 m planar path length, and a real
  200x360 OccupancyGrid at 0.1 m resolution.
- Final timing was 14.022 simulated seconds over 463.149 wall seconds
  (average RTF 0.030275). Completion method remained null; the run was stopped
  as a blocked diagnostic run, not declared successful.
- DSV frontier diagnostics still recorded zero explicit frontier clusters in
  the observed window, and the rear-waypoint heading remained unstable across
  replans. These are blocking, not residual non-blocking risks.
- Because full exploration and repeatability gates did not pass, governance
  initially prohibited local `master` merge. On 2026-07-28, the user explicitly
  overrode that merge decision while accepting the documented limitations.

## Skill evidence

- `project-governance` created the isolated branch/worktree, backed up the
  dirty root state, constrained the diff, enforced the initial no-merge
  decision, and recorded the later explicit manual merge authorization.
- `cheap-code-worker` was invoked through its required local Claude CLI flow.
  Its usable patch proposal was manually audited; proposed tests/patch details
  were not accepted blindly, and the final runtime acceptance remained FAIL.
