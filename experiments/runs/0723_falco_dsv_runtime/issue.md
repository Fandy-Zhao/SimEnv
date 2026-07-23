# Issue: FALCO + DSV Real Runtime Validation

Date: 2026-07-23

Branch: `feat/0723-falco-dsv-navigation-integration`

Baseline: `01776ecdb45cd00dfcaafcde0b89699f0444f4a4`

## Goal

Validate the real ROS runtime chain:

Gazebo -> LiDAR -> FAST-LIO2 -> DSV-Planner -> FALCO ->
`simenv_navigation_bridge` -> Unitree Trotting.

This task validates real interfaces, safety gates, and low-speed closed-loop
behavior. It does not optimize RTF and does not merge to `master`.

## Scope

- Audit actual source and launch interfaces.
- Rebuild with `./tools/build_with_venv.sh`.
- Run staged R0-R6 validation, stopping before higher-risk motion when an
  earlier gate fails.
- Add reproducible runtime test tooling only if needed.
- Record evidence under `experiments/runs/0723_falco_dsv_runtime/`.
- Update project status documents and final report.

## Non-Scope

- Gazebo collision geometry, building generation, physics profile, robot
  dynamics, Unitree Trotting core, RL controller/policy, FAST-LIO2 core, LiDAR
  model/mount, and scoring logic.
- Remote push, merge to `master`, history rewrite, or root workspace cleanup.

## Acceptance Criteria

- R0 and R1 pass.
- R2 real FAST-LIO2 data and TF are audited.
- R3 proves FALCO receives real data while `/cmd_vel` remains safe when
  navigation is disabled.
- R4/R6 motion tests run only after their prerequisites pass.
- Stop/timeout/state-gate behavior is verified before claiming closed-loop
  readiness.

## Risks

- Gazebo/FAST-LIO2 runtime may be timing limited.
- Existing `auto.sh` performs broad cleanup; use only inside the task worktree
  and do not touch the dirty root workspace.
- Real DSV waypoint generation may be blocked by map readiness or single-floor
  constraints.
