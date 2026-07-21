# Task Report: `earth.world` Flat-Ground Fix

## Branch

- `fix/0720-earth-flat-ground`
- Worktree: `/home/zzf/search_ws/SimEnv_worktrees/earth-flat-ground`
- Base: `2909093d68dbddf135575147198e32cf95bd5f01`

## Summary

`earth.world` now provides a minimal flat benchmark world for `WORLD_MODE=earth`: existing physics, scene, `sun`, and a single `ground_plane` include. The two raised inline box platform models were removed completely, including visual and collision geometry.

The benchmark spawn remains `x=0.0 y=0.0 z=0.6 roll=0 pitch=0 yaw=0.0`; this avoids using spawn height as a workaround and keeps the known Unitree A1 earth launch height.

## Files Changed

- `src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world`
  - Removed `platform_1` and `platform_2` complete inline model blocks.
- `PROJECT_STATE.md`
  - Recorded the active flat-ground fix and runtime validation gap.
- `CHANGELOG.md`
  - Added the user-visible earth flat-ground fix entry.
- `docs/module_status.md`
  - Updated `unitree_guide` status and validation notes.
- `experiments/runs/0720_earth-flat-ground/issue.md`
  - Captured scope, non-scope, acceptance criteria, and risks.
- `experiments/runs/0720_earth-flat-ground/notes.md`
  - Captured evidence, spawn pose, and validation results.

## Tests

- `python3` XML parse: PASS
- XML content check for platform removal, one ground plane include, default `WORLD_MODE=competition`, and earth path resolution: PASS
- `gz sdf -k src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world`: PASS
- `git diff --check`: PASS
- `bash -n auto.sh`: PASS
- `timeout --foreground 15s gzserver --verbose src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world`: timed out as expected while server remained running; no parse/load error in captured log tail.

## Documentation Updated

- `PROJECT_STATE.md`
- `CHANGELOG.md`
- `docs/module_status.md`
- `experiments/runs/0720_earth-flat-ground/issue.md`
- `experiments/runs/0720_earth-flat-ground/notes.md`

## Git

- Commit: recorded in the final task response after amend
- Diff stat at commit:
  - 8 files changed
  - 255 insertions
  - 121 deletions
- No merge performed.

## Risks

- Full A1 spawn, FixedStand tilt, base height, base angular velocity, and four-foot contact validation was not completed in this isolated worktree because it lacks `devel/setup.bash`.
- `ground_plane` is a Gazebo model include; static validation confirms the include count and absence of inline collisions, not the expanded model internals.
- This fix removes benchmark platforms. Any future platform/stair experiment should use a separate world file.

## Next Step

Run `WORLD_MODE=earth` in a built worktree or with a known-good overlay and repeat FixedStand plus RL motion smoke, recording base height, tilt, angular velocity, foot positions, and initial penetration/contact evidence.
