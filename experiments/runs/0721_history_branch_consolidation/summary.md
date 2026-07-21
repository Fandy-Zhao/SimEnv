# History Branch Consolidation — Governance Report

**Date:** 2026-07-21
**Governance branch:** `governance/0721-history-branch-consolidation`
**Governance worktree:** `/home/zzf/search_ws/SimEnv_worktrees/history-consolidation`

---

## 1. Verdict

**HISTORY_CONSOLIDATION_PASS**

All historical branches audited. Two validated code fixes integrated into master. One fix already present. All non-code commits (diagnostic evidence, experiment data, reports) preserved on their branches but excluded from merge.

---

## 2. Governance Baseline

| Field | Value |
|-------|-------|
| Original master HEAD | `2993a09b18384eebc92c2fecc045c2c4349d451d` |
| Final master HEAD | `8df36bf64e7a340bb9a15f157c1493982e1bd4c5` |
| Governance branch | `governance/0721-history-branch-consolidation` |
| Governance worktree | `/home/zzf/search_ws/SimEnv_worktrees/history-consolidation` |
| Root workspace branch | `backup/0720-root-uncommitted-state` |
| Root workspace HEAD | `05da43c90af82fcb392bd76997f8b4ff6e6ca8e6` |
| Root workspace preserved | YES — not modified, branch not switched |

---

## 3. Worktree Audit

| Path | Branch | HEAD | State | Disposition |
|------|--------|------|-------|-------------|
| `/home/zzf/search_ws/SimEnv` | `backup/0720-root-uncommitted-state` | `05da43c9` | **DIRTY** (16 mod, 15067 untracked) | BLOCKED — preserved as-is |
| `.../SimEnv-earth-physics-profile` | `feat/earth-world-physics-profile` | `13c9ed1d` | CLEAN | CLEAN_MERGED |
| `.../SimEnv-earth-physics-runtime` | `fix/earth-physics-profile-runtime-validation` | `29141880` | CLEAN | CLEAN_MERGED |
| `.../SimEnv-earth-rtf-diagnosis` | `diagnose/earth-world-a1-rtf` | `c6383e87` | CLEAN | CLEAN_MERGED |
| `.../SimEnv-earth-world-clean` | `feat/0720-earth-world-motion-benchmark-clean` | `ee800833` | **DIRTY** (4 mod) | DIRTY_TRACKED |
| `.../earth-flat-ground` | `fix/0720-earth-flat-ground` | `5f5f9045` | CLEAN | CLEAN_MERGED |
| `.../earth-rl-motion` | `test/0720-earth-flat-ground-runtime` | `1cd32e0c` | CLEAN | CLEAN_MERGED |
| `.../earth-rl-motion-validation` | `test/0720-earth-rl-motion-validation` | `2909093d` | **DIRTY** (4 mod + untracked) | DIRTY_TRACKED |
| `.../earth-rtf-regression` | `diagnose/0720-earth-rtf-regression` | `2909093d` | **DIRTY** (4 mod + untracked) | DIRTY_TRACKED |
| `.../fastlio2-pointcloud-frame-semantics` | `fix/0718-fast-lio2-pointcloud-frame-semantics` | `2cd801de` | CLEAN | CLEAN_MERGED |
| `.../g2-baseline-integration` | `test/0718-g2-trotting-motion-baseline` | `af99255b` | CLEAN | CLEAN_UNMERGED (experiment) |
| `.../g2-fall-validator-frame-semantics` | `fix/0719-g2-fall-validator-frame-semantics` | `2ac4cd0d` | CLEAN | CLEAN_UNMERGED (experiment) |
| `.../g2-pre-wave-block-reason` | `diagnose/0719-g2-pre-wave-block-reason` | `fc717163` | CLEAN | CLEAN_UNMERGED (experiment) |
| `.../history-consolidation` | `governance/0721-history-branch-consolidation` | `b6b7a209` | CLEAN | GOVERNANCE |
| `.../integration` | `integration/0717-mapping-stage2` | `0b434920` | CLEAN | CLEAN_MERGED |
| `.../master-merge` | `master` | `8df36bf6` | CLEAN | MASTER (final) |
| `.../motion-recovery-g1` | `fix/0717-g1-fixed-sim-scheduler` | `5da49db7` | CLEAN | CLEAN_UNMERGED (diagnostic) |
| `.../repository-consolidation` | `integrate/0720-validated-changes` | `764e0124` | CLEAN | CLEAN_MERGED |
| `.../rl-fast-validation` | `test/0721-rl-fast-validation` | `ccd144ab` | CLEAN | CLEAN_MERGED |
| `.../runtime-pointcloud-orientation` | `fix/0718-runtime-pointcloud-orientation` | `5325f034` | CLEAN | CLEAN_MERGED |
| `.../stage2` | `feat/0717-fastlio2-stage2` | `f105faf7` | CLEAN | CLEAN_MERGED |
| `.../trot-rl` | `exp/0717-trot-rl-floor-mapping` | `1d8b9488` | CLEAN | CLEAN_MERGED |
| `.../trot-rl-speed` | `exp/0717-trot-rl-speed-profile` | `ed1938d9` | CLEAN | CLEAN_MERGED |
| `.../unitree-runtime-rebuild` | `fix/0721-unitree-runtime-rebuild-and-retest` | `3f322a21` | CLEAN | CLEAN_MERGED |

---

## 4. Branch Audit — Non-Merged Branches

| Branch | HEAD | Unique commits | Classification | Action |
|--------|------|----------------|----------------|--------|
| `backup/0720-root-uncommitted-state` | `05da43c9` | 3 (+1 merge) | BACKUP_ONLY | preserve |
| `diagnose/0719-g2-pre-wave-block-reason` | `fc717163` | 3 | EXPERIMENT_ONLY | preserve |
| `diagnose/0719-g2-pre-wave-numerical-validity` | `b499407d` | 2 | EXPERIMENT_ONLY | preserve |
| `exp/0715-gazebo-rtf-diagnosis` | `336d04a1` | 2 | EXPERIMENT_ONLY | preserve |
| `exp/0715-trotting-rl-cmd-test` | `053ea13d` | 3 (1 fix already on master) | PARTIALLY_MERGED | preserve |
| `feat/0720-earth-world-motion-benchmark` | `b499407d` | 2 | EXPERIMENT_ONLY | preserve |
| `fix/0712-catkin-build-repair` | `c2389419` | 2 (extracted code fixes) | **INTEGRATED** | merged |
| `fix/0717-g1-fixed-sim-scheduler` | `5da49db7` | 6 | DIAGNOSTIC_ONLY | preserve |
| `fix/0717-motion-capability-recovery` | `05d69a8a` | 2 | DIAGNOSTIC_ONLY | preserve |
| `fix/0719-g2-fall-validator-frame-semantics` | `2ac4cd0d` | 2 | EXPERIMENT_ONLY | preserve |
| `fix/0720-earth-flat-ground` | `5f5f9045` | 0 | PATCH_EQUIVALENT | exclude |
| `test/0718-g2-trotting-motion-baseline` | `af99255b` | 2 | EXPERIMENT_ONLY | preserve |

---

## 5. Included Commits

| Source | Files | Method |
|--------|-------|--------|
| `165903ce` (fix/0712-catkin-build-repair) | `src/uav_simulator/mockamap/src/ces_randommap.cpp` | reconstructed (git diff + git apply) |
| `c2389419` (fix/0712-catkin-build-repair) — csv_reader.hpp only | `src/Mid360_imu_sim/include/livox_laser_simulation/csv_reader.hpp` | reconstructed (auto.sh portion superseded) |

### Fix details:
1. **ces_randommap.cpp:** Added `#include <deque>` (line 24), qualified `std::deque<...>` (line 57)
2. **csv_reader.hpp:** Replaced `while(!file_stream.eof())` with `while(std::getline(file_stream, line_str))` + empty line skip

---

## 6. Excluded Commits (key ones)

| Commit | Reason |
|--------|--------|
| `053ea13d` (fix: guard RL entry) | Already present on master (verified in State_RL_test.cpp lines 42-49) |
| `5f5f9045` (fix: earth flat ground) | Cherry-equivalent in master |
| `1a524244`, `d2492470` | Cherry-equivalent in master |
| `e5e27cfe`, `b499407d`, `af99255b`, `fc717163`, `2ac4cd0d` | Experiment data / diagnostic reports |
| `f489e553` (wip) | WIP snapshot with generated files |
| `98c42df5` through `5da49db7` (6 commits) | Controller diagnostic instrumentation |
| `6376d16d`, `336d04a1` | Documentation / .gitignore only |

---

## 7. Dirty Changes

| Worktree | Key Changes | Recommendation |
|----------|-------------|----------------|
| Root (`SimEnv`) | earth.world platforms, regenerated building, Gazebo logs | User decides |
| `earth-world-clean` | WORLD_MODE doc updates in CHANGELOG/PROJECT_STATE/module_status | Commit or discard |
| `earth-rl-motion-validation` | AUTO_FIXED_STAND validation docs | Commit or discard |
| `earth-rtf-regression` | RTF diagnosis docs | Commit or discard |

---

## 8. Validation

| Check | Result |
|-------|--------|
| `git diff --check` | PASS |
| `bash -n auto.sh` | PASS |
| `#include <deque>` + `std::deque` in ces_randommap.cpp | PASS |
| csv_reader.hpp no `eof()`, uses `while(std::getline())` | PASS |
| `tools/build_with_venv.sh` | PASS (100%, all targets) |
| PHYSICS_PROFILE in auto.sh (23 occurrences) | PASS |
| Candidate C profile (0.001/1000/20) | CONFIRMED |
| World-mode isolation (earth vs competition) | CONFIRMED |
| Gazebo runtime verification | NOT PERFORMED |
| Trotting RTF | PENDING |

---

## 9. Final Root Workspace State

| Field | Value |
|-------|-------|
| Path | `/home/zzf/search_ws/SimEnv` |
| Branch | `backup/0720-root-uncommitted-state` |
| HEAD | `05da43c90af82fcb392bd76997f8b4ff6e6ca8e6` |
| Clean? | NO |
| Switched to master? | NO — **BLOCKED_BY_DIRTY_WORKSPACE** |
| Blocking reason | 16 modified tracked files prevent branch switch |

---

## 10. Remaining Risks

- Trotting RTF: **PENDING** (runtime verification not performed)
- Root workspace: dirty, user must decide disposition
- 3 worktrees with uncommitted documentation updates
- Gazebo startup chain not smoke-tested in live environment
- 2 stashes preserved (auto.sh backup + old WIP)

---

## Answer Summary

| # | Question | Answer |
|---|----------|--------|
| 1 | Final master HEAD | `8df36bf64e7a340bb9a15f157c1493982e1bd4c5` |
| 2 | Effective changes in master | 2 code fixes (deque include + CSV reader EOF bug) |
| 3 | Method | Reconstructed via `git diff \| git apply` |
| 4 | Excluded as duplicate/preexisting | 1 fix (RL guard) already on master; `fix/0720-earth-flat-ground` cherry-equivalent |
| 5 | Dirty worktrees? | Root + 3 doc worktrees |
| 6 | Root switched to master? | NO |
| 7 | Blocking reason | 16 modified tracked files |
| 8 | PHYSICS_PROFILE findable? | YES — 23 occurrences in auto.sh on master |
| 9 | Compile test? | PASS (100%, all targets built) |
| 10 | Trotting RTF? | **PENDING** |
