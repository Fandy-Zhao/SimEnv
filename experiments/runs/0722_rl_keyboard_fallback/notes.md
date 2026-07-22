# Notes: RL Keyboard Command Fallback

## Implementation

- Added `State_RL::resolveCommandSnapshot()`.
- Fresh `/cmd_vel` remains the highest-priority runtime command source.
- If `/cmd_vel` is missing or older than 0.5 s, RL maps keyboard `userValue`
  into the same command axes used by Trotting.
- Non-finite `/cmd_vel` values are rejected and converted to a stop command.
- The default RL policy path is set to the Earth flat-ground recommended
  `src/unitree_guide/logs/policy_act_inference_plane.pt`; runtime policy
  override priority is unchanged.

## Validation

- `git diff --check`: PASS.
- `/home/zzf/search_ws/SimEnv/tools/build_with_venv.sh`: PASS; rebuilt
  `devel/lib/unitree_guide/junior_ctrl`.

## Notes

- The default policy change was present before the keyboard fallback commit and
  is intentionally preserved in this branch before merging back to `master`.
