# Runtime Pointcloud Orientation Notes

Date: 2026-07-18
Branch: `fix/0718-runtime-pointcloud-orientation`
Worktree: `/home/zzf/search_ws/SimEnv_worktrees/runtime-pointcloud-orientation`

## Baseline

- IDE/root worktree: `/home/zzf/search_ws/SimEnv`
- IDE/root branch: `test/0718-g2-trotting-motion-baseline`
- IDE/root HEAD: `a2e00509`
- `git merge-base --is-ancestor 69ff34e7 HEAD`: failed in the IDE/root worktree.
- Initial unsourced shell:
  - `ROS_PACKAGE_PATH=/opt/ros/noetic/share`
  - `CMAKE_PREFIX_PATH=/opt/ros/noetic`
  - `rospack find simenv_fast_lio2_integration`: not found
  - `rospack find a1_description`: not found

## Isolation

- Created task worktree from `master`:
  `/home/zzf/search_ws/SimEnv_worktrees/runtime-pointcloud-orientation`
- Branch: `fix/0718-runtime-pointcloud-orientation`
- HEAD: `6a4f1124`
- `git merge-base --is-ancestor 69ff34e7 HEAD`: pass.
- Root/G2 dirty worktree was not modified.

## Worker Static Inventory

`cheap-code-worker` wrote
`experiments/runs/0718_runtime-pointcloud-orientation/worker_static_inventory.md`.
Codex reviewed it and repeated the search. No launch, shell, YAML, xacro, or
Python file explicitly sets legacy `rotation_y_deg=-90` or
`rotation_x_deg=180`.

## Runtime Findings

- A stale root-worktree process was found before clean startup:
  `/home/zzf/search_ws/SimEnv/.venv/bin/python /home/zzf/search_ws/SimEnv/devel/lib/unitree_guide/pointcloud2livox.py`.
  It was stopped before isolated validation.
- Manual ROS launch without `auto.sh` environment cleanup failed because xacro
  imported Miniconda Python 3.13 `xml.dom.minidom`, proving IDE/Python
  contamination can break runtime launch.
- Clean environment launch used:
  - system PATH only;
  - `PYTHONNOUSERSITE=1`;
  - task `devel/setup.bash`;
  - task `ROS_PACKAGE_PATH` before the root external FAST-LIO2 source.

## Positive Evidence

`runtime_smoke_check_with_registered_pass.log`:

- Adapter process:
  `/usr/bin/python3 /home/zzf/search_ws/SimEnv_worktrees/runtime-pointcloud-orientation/src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py`
- Adapter private params: `{}`; no rotation params active.
- `git_head=6a4f1124 contains_69ff34e7=true`
- `/scan` and `/scan_pointcloud2`:
  - same timestamp;
  - same frame: `laser_livox`;
  - same point count: `24000`;
  - compared `24000` point triples;
  - max absolute coordinate error: `0`.
- TF `base -> laser_livox`:
  - translation `[0.2, 0.0, 0.08]`;
  - LiDAR local `+X` in base `[0.707388, 0.0, -0.706825]`.
- `/cloud_registered`:
  - frame `camera_init`;
  - low-plane normal angle from `+Z`: `0.550 deg`.

## Conclusion

The runtime adapter in the corrected worktree is a pure format converter. It
does not create robot `-X` points. The remaining 45-degree downward direction
belongs to the physical `base -> laser_livox` mount. FAST-LIO2 registered cloud
output is already close to horizontal.
