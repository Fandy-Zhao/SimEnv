# Issue: G2 Trotting Motion Baseline

## Goal

Complete G2-B Trotting baseline evidence for Unitree A1 in the SimEnv ROS1
Noetic + Gazebo Classic runtime, then enter G2-R only if the completed baseline
supports one primary root cause and a targeted fix.

## Scope

- G2-B runtime configuration freeze.
- Isolated trial runner and serial matrix orchestration.
- Ground-truth, controller timing, foot-force, joint-state, event, manifest,
  and metric evidence capture.
- `vx=0.00`, `0.10`, `0.30`, and `0.50 m/s`, at least 3 epochs per speed.
- Sim-time phase boundaries, steady-window metrics, stop-response metrics,
  validity classification, aggregation, and report updates.

## Non-Scope

- Trotting Kp/Kd, BalanceCtrl, gait period, stance ratio, foot trajectory,
  command scale, Estimator model, scheduler, contact threshold, URDF/SDF,
  mass/inertia, collision, friction, ODE/physics, RL policy, observation, or
  action changes during G2-B.
- Automatic push, PR creation, build/devel submission, or large raw CSV commits.

## Acceptance Criteria

- Required governance and evidence documents exist under
  `docs/active/0718-g2-trotting-motion-baseline/`.
- Trial tooling can run an isolated baseline trial and preserve all required
  compact evidence files.
- Pure metric helpers pass unit tests for transforms, windows, stop detection,
  invalid filtering, median aggregation, and drift ratio.
- Baseline verdict uses only valid trials and reports invalid evidence instead
  of deleting or retrying away failures.

## Risks

- Existing worktree has unrelated generated/log/result changes and untracked
  external packages.
- Low RTF can make the full matrix expensive in wall-clock time.
- Runtime trial execution can alter `generated_building/`, `logs/`, and
  `results/`; G2 evidence copies live under this run directory.

## Expected Modules

- `docs/active/0718-g2-trotting-motion-baseline/`
- `experiments/runs/0718_g2_trotting_motion_baseline/`
