# Baseline

- Date: 2026-07-23
- Root worktree: `/home/zzf/search_ws/SimEnv`
- Task worktree: `/home/zzf/search_ws/SimEnv_worktrees/fast-lio2-tf-fix`
- Task branch: `fix/0723-fast-lio2-tf-repeated-data`
- Task base: `cf75e8305965d44b221876ea2c2c019ff3d16906`
- Local `master`: `cf75e8305965d44b221876ea2c2c019ff3d16906`

## Root Worktree

- Branch: `exp/0722-earth-rl-speed-0to1`
- HEAD: `bfbbce24078e5f3d0c40b2146360d55246226153`
- Status: dirty generated scene, logs, and results files were observed and intentionally preserved.

## Initial Plan

1. Build an isolated task worktree from local `master`.
2. Record static TF publisher evidence and launch ownership.
3. Attempt runtime reproduction with the repository startup path.
4. Determine primary root cause and non-causes.
5. Apply the smallest code/configuration change supported by evidence.
6. Run syntax/build/smoke checks that are available in this environment.
