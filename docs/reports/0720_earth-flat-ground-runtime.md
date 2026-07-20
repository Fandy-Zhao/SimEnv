# Earth Flat-Ground Runtime Validation

## Verdict

`TASK_PARTIAL`

Commit `5f5f9045` is integrated into a runnable worktree and the platform removal is confirmed at runtime, but the validation cannot proceed to Earth E0 or RL. G1 initial-contact/entry validation fails, and the C0 competition FixedStand control rerun also fails, so the required root-cause closure is not established.

## Branch / Worktree / Base

- Branch: `test/0720-earth-flat-ground-runtime`
- Worktree: `/home/zzf/search_ws/SimEnv_worktrees/earth-rl-motion`
- Base HEAD: `63119f892cec8c1e20d21a48df7c6593d9156fad`
- Runtime cherry-pick HEAD before local tooling: `2478e8cbc72e7decba8d78a3e598f58495c2037c`
- Cherry-picked source commit: `5f5f9045cb760292a389f25cba023bde53675723`

## Fix Commit Verification

`git show --stat --oneline 5f5f9045` reports:

```text
8 files changed, 212 insertions(+), 75 deletions(-)
```

The `earth.world` portion removes the complete inline `platform_1` and `platform_2` model blocks, including visual and collision box geometry. `ground_plane` remains as a single include. The earlier flat-ground report used a different amended-commit stat in one place; this report treats `git show --stat 5f5f9045` as authoritative.

## Build / Runtime Environment

- `ROS_PACKAGE_PATH=/home/zzf/search_ws/SimEnv_worktrees/earth-rl-motion/src:/opt/ros/noetic/share`
- `CMAKE_PREFIX_PATH=/home/zzf/search_ws/SimEnv_worktrees/earth-rl-motion/devel:/opt/ros/noetic`
- `rospack find unitree_guide`: `/home/zzf/search_ws/SimEnv_worktrees/earth-rl-motion/src/unitree_guide/unitree_guide/unitree_guide`
- `rospack find unitree_gazebo`: `/home/zzf/search_ws/SimEnv_worktrees/earth-rl-motion/src/unitree_guide/unitree_ros/unitree_gazebo`
- `junior_ctrl` sha256: `997538cf0c0cf3faa9c38e88a29cee5694a26b5f35541897c0d521707149ab1a`
- `state_from_gazebo` sha256: `4ff3ac9efb36f48ca8b0752f819a7704219868558378c11f9c144972a743813e`
- Earth spawn pose: `x=0.0 y=0.0 z=0.6 roll=0 pitch=0 yaw=0.0`

## G0 World Runtime

- Verdict: `G0_PASS`
- Runtime model list: `ground_plane`
- `platform_1`: absent
- `platform_2`: absent
- `ground_plane`: present
- RTF median: `0.9833421436430887`
- Note: source `earth.world` retains the `sun` include, but Gazebo Classic did not publish `sun` as a normal `/gazebo/model_states` model.

## G1 Initial Contact

- Official controller-epoch case: `G1_spawn_contact_controller`
- Verdict: `G1_FAIL_ATTITUDE`
- Runtime model list: `a1_gazebo`, `ground_plane`
- Max tilt: `5.188577548306383 deg`
- P95 tilt: `3.8775961305557307 deg`
- Min base height: `0.0504415113362311 m`
- Final base height: `0.05603132338577144 m`
- Peak angular velocity: `0.13637924338501797 rad/s`
- RTF median: `0.988073959106661`

Limited conclusion: no platform collision remains and no angular explosion appears in the initial window, but the base settles to near-ground height before FixedStand. Direct contact sensors were not available, so this does not prove exact foot penetration status.

## C0 Competition FixedStand

- Official rerun case: `C0_competition_fixedstand_rerun`
- Verdict: `FAIL_ATTITUDE`
- Final FSM state: `2`
- Max tilt: `170.68191595126532 deg`
- P95 tilt: `170.10882628389336 deg`
- Min base height: `0.07887235055623487 m`
- Forward displacement: `-0.2766497572662883 m`
- Lateral displacement: `-0.005954142597455566 m`
- Yaw change: `111.20017844126572 deg`
- RTF median: `0.5872710468910193`

This does not match the earlier competition reference (`max_tilt_deg ≈ 3.09`), so the current runtime artifact/entry chain is not a valid no-regression control.

## Earth E0 / RL Gates

- Earth E0 three-run FixedStand: `BLOCKED_BY_G1`
- E1 RL zero: `BLOCKED_BY_E0`
- E2 RL `vx=0.05`: `BLOCKED_BY_E0`
- E3 RL `vx=0.10`: `BLOCKED_BY_E0`
- E5 stop response: `BLOCKED_BY_E0`

No RL performance conclusion is made.

## Before / After Tilt

- Old Earth failure: `max_tilt_deg ≈ 169.84`, fall around 12 s simulation time.
- New Earth E0: not run because G1 failed.
- New G1 initial-contact max tilt is low (`5.19 deg`) but base height is near ground (`0.050 m`).
- C0 competition rerun also tilts to `170.68 deg`.

## Root Cause Closure

Not confirmed. The platform models are gone at runtime, but the required evidence chain fails because G1 does not pass and C0 regresses. The safe interpretation is:

```text
PLATFORM_REMOVAL_INTEGRATED_RUNTIME_PLATFORM_ABSENT_BUT_VALIDATION_BLOCKED
```

## RTF / CPU

RTF median:

- G0: `0.9833421436430887`
- G1 controller epoch: `0.988073959106661`
- C0 rerun: `0.5872710468910193`

C0 is above the hard `0.5` median threshold but much lower than the previous `~1.00` reference. No RL performance case was run.

## cheap-code-worker Usage

`cheap-code-worker` was used only to create the governed experiment scaffold under `experiments/runs/0720_earth-flat-ground-runtime/`. Codex reviewed the result and then implemented and validated the runtime scripts. The worker did not modify world geometry, controller, policy, URDF/xacro, physics, spawn height, or launch behavior.

## Tests / Build

Passed:

- `bash -n auto.sh`
- `bash -n experiments/runs/0720_earth-flat-ground-runtime/scripts/*.sh`
- `python3 -m py_compile experiments/runs/0720_earth-flat-ground-runtime/scripts/*.py`
- `python3 -m unittest discover -s experiments/runs/0720_earth-flat-ground-runtime/tests -p 'test_*.py'`
- Existing earth RL metric/static tests
- `earth.world` XML parse
- `gz sdf -k earth.world`
- `git diff --check`

Build blocked:

- Torch-enabled rebuild failed because CUDA `nvcc` cannot execute `cc1plus`.
- Full `catkin_make -j` failed for the same CUDA/Torch configuration probe.

## Stair Policy Follow-Up

The user requested a later run with `/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_stair.pt`. That policy was not switched in this run because RL is gated behind E0 and E0 was blocked. Existing results are preserved under separate case IDs.

## Remaining Risks

- G1 and C0 failures may indicate a runtime artifact, entry-command, or controller-state issue independent of the removed platforms.
- Direct foot contact/penetration topics were not available; contact conclusions rely on model pose, base height, joint states, and model list evidence.
- The build cache remains unable to regenerate Torch-enabled artifacts due CUDA host compiler failure.
