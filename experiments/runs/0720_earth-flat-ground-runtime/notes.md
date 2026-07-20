# Runtime Notes

## Branch and Integration

- Branch: `test/0720-earth-flat-ground-runtime`
- Base HEAD before cherry-pick: `63119f892cec8c1e20d21a48df7c6593d9156fad`
- Cherry-picked source commit: `5f5f9045cb760292a389f25cba023bde53675723`
- Runtime cherry-pick commit: `2478e8cbc72e7decba8d78a3e598f58495c2037c`
- Worktree: `/home/zzf/search_ws/SimEnv_worktrees/earth-rl-motion`

## Commit Verification

- `5f5f9045` actual stat: 8 files changed, 212 insertions, 75 deletions.
- `earth.world` diff removes complete `platform_1` and `platform_2` model blocks.
- Runtime G0 model list contains only `ground_plane`; both platform names are absent.
- Source `earth.world` still contains the `sun` include, but Gazebo Classic did not publish `sun` in `/gazebo/model_states`.

## Build Artifact Source

- `devel/setup.bash`: existing in the selected worktree.
- `junior_ctrl`: `devel/lib/unitree_guide/junior_ctrl`
  - sha256 `997538cf0c0cf3faa9c38e88a29cee5694a26b5f35541897c0d521707149ab1a`
- `state_from_gazebo`: `devel/lib/unitree_guide/state_from_gazebo`
  - sha256 `4ff3ac9efb36f48ca8b0752f819a7704219868558378c11f9c144972a743813e`
- Default plane policy:
  - `src/unitree_guide/logs/policy_act_inference_plane.pt`
  - sha256 `e886847fe266e3c2f7c08825fceeaecfa75c7eac5f780b25b6d4dca173ff8bef`
- User-requested later stair policy is present and was not used in this gated run:
  - `/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_stair.pt`
  - worktree copy `src/unitree_guide/logs/policy_act_inference_stair.pt`
  - sha256 `2d5aa72511c0c6609c02f4105845eee6974d3d73431497f8f35306da9588fe14`

## Runtime Results

- G0: `G0_PASS`
  - model list: `ground_plane`
  - `platform_1=false`, `platform_2=false`
  - RTF median `0.9833421436430887`
- G1 official controller epoch: `G1_FAIL_ATTITUDE`
  - model list: `a1_gazebo`, `ground_plane`
  - min base height `0.0504415113362311 m`
  - max tilt `5.188577548306383 deg`
  - peak angular velocity `0.13637924338501797 rad/s`
  - RTF median `0.988073959106661`
  - Interpretation: no platform remains and no angular explosion was observed, but the base settles to near-ground height before FixedStand. Direct contact sensors were not available, so this is limited evidence of body/ground contact rather than a precise penetration measurement.
- C0 rerun: `FAIL_ATTITUDE`
  - final FSM state `2`
  - max tilt `170.68191595126532 deg`
  - min base height `0.07887235055623487 m`
  - RTF median `0.5872710468910193`
  - Interpretation: the selected runtime artifact/entry chain does not reproduce the previous competition FixedStand reference, so root-cause closure cannot be claimed.

## Gate Decisions

- Earth E0 was not run because G1 did not pass.
- RL E1/E2/E3/E5 were not run because E0 was blocked.
- The requested later stair-policy test is recorded as a future separate phase; no policy file was switched in this run.

## Build Results

- Static checks and Python tests passed.
- Torch-enabled rebuild failed at CUDA compiler probe: `/usr/local/cuda-11.8/bin/nvcc` cannot execute `cc1plus`.
- Full `catkin_make -j` failed for the same CUDA/Torch cache reason.
- These build failures match existing project risk and are unrelated to the new benchmark scripts or `earth.world` cherry-pick.
