#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
export WORLD_MODE=competition
export PHYSICS_PROFILE=normal
export GUI=false
export START_CONTROLLER=1
export ENABLE_SENSOR_DATA=0
export ENABLE_POINTCLOUD_CONVERTER=0
export ENABLE_FAST_LIO2=0
export ENABLE_RVIZ=0
export ENABLE_REFEREE_ODOM=0
export ENABLE_GROUND_TRUTH=0
export START_BUILDING_CONTROL=0
export RL_POLICY_PATH=/home/zzf/search_ws/SimEnv_worktrees/competition-rl-rtf-collapse/src/unitree_guide/logs/policy_act_inference_stair.pt
cd "$REPO_ROOT"
bash "$REPO_ROOT/auto.sh"
