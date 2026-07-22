#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
export WORLD_MODE=earth
export PHYSICS_PROFILE=normal
export GUI=false
cd "$REPO_ROOT"
bash "$REPO_ROOT/auto.sh"
