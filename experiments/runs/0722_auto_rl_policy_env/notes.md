# Notes: Explicit auto.sh RL Policy Environment Support

## Plan

- Add an `RL_POLICY_PATH` line to the startup summary.
- Export `RL_POLICY_PATH` into the terminal environment used for `junior_ctrl`
  when the variable is set.
- Keep controller-side policy resolution unchanged.

## Validation

- `bash -n auto.sh`: PASS.
- `git diff --check`: PASS.
- Startup summary smoke with
  `RL_POLICY_PATH=/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_plane.pt`
  and `START_CONTROLLER=0`: PASS; summary prints the explicit override.
- Startup summary smoke without `RL_POLICY_PATH` and `START_CONTROLLER=0`:
  PASS; summary prints that the override is unset and the controller default
  remains active unless `/rl_policy_path` is set.
- Runtime smoke runs were stopped with Ctrl-C after startup summary validation;
  generated Earth world and runtime logs were cleaned.
