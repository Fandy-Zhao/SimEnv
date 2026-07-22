# Notes: RL Keyboard Command Fallback

## Implementation

- Added `State_RL::resolveCommandSnapshot()`.
- Fresh `/cmd_vel` remains the highest-priority runtime command source.
- If `/cmd_vel` is missing or older than 0.5 s, RL maps keyboard `userValue`
  into the same command axes used by Trotting.
- Non-finite `/cmd_vel` values are rejected and converted to a stop command.

## Validation

- `git diff --check`: PASS.
- `/home/zzf/search_ws/SimEnv/tools/build_with_venv.sh`: PASS; rebuilt
  `devel/lib/unitree_guide/junior_ctrl`.

## Notes

- A pre-existing uncommitted change in `State_RL_test.cpp` changes the default
  policy path from stair to plane. It was preserved in the worktree and is not
  part of this keyboard-fallback change.
