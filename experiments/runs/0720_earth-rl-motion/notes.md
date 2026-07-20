# Earth RL Motion Notes

## Phase 0 Audit

- Root branch: `backup/0720-root-uncommitted-state`
- Root HEAD: `f489e553e06c680069c75cfbc3fdccaed184edad`
- Root dirty state: many untracked experiment, dependency, and runtime files;
  left untouched.
- Task branch: `test/0720-earth-rl-motion`
- Task worktree: `/home/zzf/search_ws/SimEnv_worktrees/earth-rl-motion`
- Base: `84ff02d7022c11dbb26fdbb2ff37322bf4aaf814`
- New worktree initial status: clean.

## Phase 1 Read-Only Review

`master@84ff02d7` did not track `earth.world`. The only discovered copy was the
root worktree's untracked
`src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world`, which was
read-only inspected and manually added to this task worktree.

Relevant old branch review:

| File | Old Branch Change | Current Master Has It | Migrated | Reason | Risk |
| --- | --- | --- | --- | --- | --- |
| `auto.sh` | Old branch listed in tree, but earth branches have no `master...clean` diff and the non-clean branch only contains excluded G2 diagnostic commits. | No earth mode. | Yes, manually. | Need isolated `WORLD_MODE=earth`. | Competition defaults must remain unchanged. |
| `src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world` | Not tracked in old earth branches. | No. | Yes, from read-only root untracked file. | Required world asset. | Spawn/contact stability unvalidated before runtime. |
| G2 diagnostic scripts/docs | Non-clean branch contains excluded diagnostic commits. | Already represented by validated master chain. | No. | Task forbids cherry-picking `b499407d`/`2ac4cd0d` or importing old G2 data. | Avoids unrelated G2 changes. |

## Static Validation

- `bash -n auto.sh`: PASS
- `bash -n run_earth_rl_trial.sh`: PASS
- `bash -n run_earth_rl_matrix.sh`: PASS
- Python compile for changed Python files: PASS
- Python unit tests: 7 tests PASS
- `earth.world` XML parse: PASS
- `git diff --check`: PASS

## Build Validation

- `catkin_make -j`: FAIL in Torch/CUDA configuration. CUDA `nvcc` could not
  execute `cc1plus` while probing libtorch CUDA support.
- `catkin_make -DUNITREE_ENABLE_TORCH_POLICY=OFF -j`: FAIL because
  `unitree_move_base` depends on unavailable `move_base_msgs`.
- `catkin_make -DUNITREE_ENABLE_TORCH_POLICY=OFF -DCATKIN_BLACKLIST_PACKAGES=unitree_move_base -j`:
  FAIL from unrelated UAV/SDK example targets, but produced the components
  required for earth smoke: `junior_ctrl`, `state_from_gazebo`,
  `libunitreeFootContactPlugin.so`, and `liblivox_laser_simulation.so`.

## Runtime Evidence

World-only topic smoke with `START_CONTROLLER=0`:

- `WORLD_MODE=earth`
- `/clock`: present
- `/gazebo/model_states`: present
- models: `ground_plane`, `platform_1`, `platform_2`, `a1_gazebo`
- `auto.sh startup complete`: yes
- Competition generation skipped: yes
- Optional nodes off by default: sensor data, PointCloud2 converter, ground
  truth, referee odom, FAST-LIO2, RViz, building controller.

E0 FixedStand:

- Trial: `E0_fixedstand`
- Requested state: FixedStand (`2`)
- Duration: `15.174 s` simulation time
- Final FSM state: `2`
- Samples: `75852`
- Initial pose: `x=-0.048625`, `y=-0.040659`, `z=0.494732`
- Final pose: `x=-0.060593`, `y=0.068811`, `z=0.494748`
- Forward displacement: `0.019650 m`
- Lateral displacement: `0.108354 m`
- Yaw change: `-4.762091 deg`
- Max roll: `91.911513 deg`
- Max pitch: `0.000774 deg`
- Min z: `0.494716 m`
- Verdict: `FAIL_ATTITUDE`

Because E0 fails, E1+ active RL trials were not executed. The current evidence
is insufficient to enter RL recovery; the first blocker is spawn/contact/pose
stability on the earth world.

## Cheap-Code-Worker Usage

Delegated one mechanical `auto.sh` world-mode wiring task. The worker produced a
partial diff. Main-agent review found it incomplete for startup summary and
earth-mode artifact isolation, then corrected the diff manually. No worker
changes touched control code, RL, FSM, or generated/user data.
