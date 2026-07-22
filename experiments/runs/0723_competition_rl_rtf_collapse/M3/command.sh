#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
export WORLD_MODE=competition
export PHYSICS_PROFILE=normal
export GUI=false
export START_CONTROLLER=1
export ENABLE_SENSOR_DATA=1
export ENABLE_POINTCLOUD_CONVERTER=1
export ENABLE_FAST_LIO2=0
export ENABLE_RVIZ=0
export ENABLE_REFEREE_ODOM=0
export ENABLE_GROUND_TRUTH=0
export START_BUILDING_CONTROL=0
cd "$REPO_ROOT"
bash "$REPO_ROOT/auto.sh"
