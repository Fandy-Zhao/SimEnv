# Issue: Earth RL Motion Benchmark

## Goal

Integrate the repository-local `earth.world` as an isolated launch mode and run
auditable RL motion checks on that terrain from `master@84ff02d7`.

## Scope

- Add `WORLD_MODE=competition|earth` selection to `auto.sh`.
- Track `earth.world` under the Unitree Gazebo world package.
- Add measurement-only earth RL benchmark helpers under this experiment run.
- Record static and runtime evidence without changing control behavior.

## Non-Scope

- No FSM, RL model, observation, action, reward, gait, IK, estimator, contact,
  fall-validator, or motor-command changes.
- No merge to `master`.
- No deletion or cleanup of root-worktree untracked data.

## Acceptance Criteria

- Default `WORLD_MODE=competition` keeps the competition scene generator path.
- `WORLD_MODE=earth` resolves the tracked `earth.world` and skips competition
  generation.
- Earth mode defaults optional mapping/competition nodes off while preserving
  environment overrides.
- Static shell/Python/XML checks pass.
- Runtime trials report PASS, FAIL, or ENV_LIMITED with logs and metrics.

## Risks

- The baseline commit did not track `earth.world`; it was only present as root
  worktree untracked data before this task.
- Gazebo may be unavailable or too slow in the current environment.
- RL may be unavailable if the local build lacks Torch policy support.

## Impacted Modules

- `auto.sh`
- `src/unitree_guide/unitree_ros/unitree_gazebo/worlds/`
- `experiments/runs/0720_earth-rl-motion/`
- Governance/status documents
