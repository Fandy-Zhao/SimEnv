# Issue: DSV goal bootstrap deadlock

## Goal

Restore a real single-floor DSV/FALCO closed loop in which a supervisor-gated
bootstrap motion grows the local graph and DSV publishes a finite, useful goal
that Graph Planner, FALCO, `/cmd_vel`, and the A1 execute.

## Scope

- `dsv_simenv.yaml` startup, bootstrap, and liveness parameters.
- DSV exploration bootstrap and bounded warm-up recovery.
- DSV planner validation of selected goals and RRT rejection diagnostics.
- Graph Planner defensive diagnostics for degenerate inputs.
- FALCO A1 navigation-profile liveness for real rear DSV goals (configuration
  only; no controller algorithm or safety-threshold changes).
- Short closed-loop validation through `tools/build_with_venv.sh` and `auto.sh`.

## Non-scope

- No FALCO safety-threshold reduction or collision-check bypass.
- No synthetic graph vertices, manual goals, manual velocity commands, or
  robot teleportation.
- No full 1800-second floor exploration.
- No unrelated generated-building, runtime-log, result, controller-code,
  physics, FAST-LIO2, or recorder changes.

## Acceptance criteria

1. Exploration starts only after `FSM=4`, navigation enabled, and the
   supervisor exploration request is confirmed.
2. Bootstrap produces at least 0.5 m measured XY displacement and remains on
   the initial floor.
3. Local graph grows beyond one vertex; RRT rejection diagnostics distinguish
   unknown from occupied Octomap rejection.
4. DSV rejects non-finite, mono-vertex, non-positive-gain, and <=0.4 m local
   exploration goals without publishing or counting them as valid.
5. Premature completion/unready planning is bounded, gets at most one explicit
   bootstrap recovery, and otherwise emits `BOOTSTRAP_EXPLORATION_FAILED`.
6. A useful DSV goal yields a Graph Planner path/waypoint, non-zero FALCO and
   `/cmd_vel`, and actual robot displacement.
7. Formal worktree build and short `auto.sh` runtime pass before merge.

## Risks

- The observed Octomap rejection may remain dominated by unknown or occupied
  cells after motion, requiring a separate map/collision-model correction.
- Low RTF can make the short closed-loop validation expensive in wall time.
- Bootstrap motion must remain collision-safe in the deterministic seed scene.

## Impacted modules

- `simenv_navigation_bringup`
- DSV `dsvp_launch` and `dsvplanner`
- DSV `graph_planner`
