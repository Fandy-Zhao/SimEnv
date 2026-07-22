#!/usr/bin/env bash
set -euo pipefail
cd /home/zzf/search_ws/SimEnv_worktrees/competition-rl-rtf-collapse
export WORLD_MODE='earth'
export PHYSICS_PROFILE='normal'
export GUI='false'
export SKIP_GLOBAL_PROCESS_CLEANUP=true
export TERMINAL_BACKEND=direct
export TIMING_DIAGNOSTICS_ENABLED='0'
export TIMING_DIAGNOSTICS_PATH='/home/zzf/search_ws/SimEnv_worktrees/competition-rl-rtf-collapse/experiments/runs/0723_competition_rl_rtf_collapse/M0/timing.csv'
bash ./auto.sh
