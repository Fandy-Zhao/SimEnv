# Issue: Explicit auto.sh RL Policy Environment Support

## Goal

Make `auto.sh` visibly support the RL policy override path used by
`State_RL`, so operators can see which environment override will be passed to
`junior_ctrl`.

## Scope

- Print the `RL_POLICY_PATH` override in the startup summary.
- Explicitly export `RL_POLICY_PATH` into the dedicated controller terminal
  environment when it is set.
- Preserve the existing controller-side priority:
  `/rl_policy_path` -> `RL_POLICY_PATH` -> stair default.

## Non-scope

- No policy file changes.
- No RL controller behavior changes.
- No navigation algorithm integration.
- No control parameter tuning.

## Acceptance Criteria

- `bash -n auto.sh` passes.
- Running `auto.sh` with `START_CONTROLLER=0` and `RL_POLICY_PATH=...` shows
  the override in the startup summary.
- Running `auto.sh` without `RL_POLICY_PATH` shows that the override is unset
  and the controller default remains in effect.

## Risks

- The ROS param `/rl_policy_path` is read by `junior_ctrl`, so it can still
  override the environment value even though `auto.sh` can only summarize the
  environment setting.

## Impacted Modules

- `auto.sh`
- `unitree_guide` runtime startup behavior
