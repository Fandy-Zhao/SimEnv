#!/usr/bin/env bash
set -euo pipefail

# build_rl_fast.sh — Thin build wrapper for the RL fast validation experiment
#
# Sets SIMENV_CATKIN_WHITELIST to the Unitree RL runtime profile, logs command
# and timestamps to the experiment's raw/build directory, calls
# tools/build_with_venv.sh, and propagates its exit code.
#
# Usage:
#   SIMENV_CC=/usr/bin/gcc-11 SIMENV_CXX=/usr/bin/g++-11 ./tools/build_rl_fast.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RUN_DIR="$REPO_ROOT/experiments/runs/0721_rl-fast-validation"
BUILD_LOG_DIR="$RUN_DIR/raw/build"
BUILD_LOG="$BUILD_LOG_DIR/build_rl_fast.log"

mkdir -p "$BUILD_LOG_DIR"

# ---------------------------------------------------------------------------
# Set whitelist for Unitree RL runtime profile
# ---------------------------------------------------------------------------
export SIMENV_CATKIN_WHITELIST="unitree_legged_msgs;unitree_guide;unitree_legged_control;unitree_gazebo"

# ---------------------------------------------------------------------------
# Log invocation
# ---------------------------------------------------------------------------
{
    echo "========================================"
    echo "build_rl_fast.sh invocation"
    echo "Timestamp (UTC): $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "Timestamp (epoch): $(date +%s)"
    echo "SIMENV_CATKIN_WHITELIST: $SIMENV_CATKIN_WHITELIST"
    echo "CC: ${SIMENV_CC:-<default>}"
    echo "CXX: ${SIMENV_CXX:-<default>}"
    echo "CUDA_HOME: ${CUDA_HOME:-<not set>}"
    echo "Command: $0 $*"
    echo "========================================"
} >> "$BUILD_LOG"

# ---------------------------------------------------------------------------
# Delegate to build_with_venv.sh
# ---------------------------------------------------------------------------
BUILD_EXIT=0
"$SCRIPT_DIR/build_with_venv.sh" "$@" >> "$BUILD_LOG" 2>&1 || BUILD_EXIT=$?

{
    echo ""
    echo "========================================"
    echo "build_rl_fast.sh finished"
    echo "Exit code: $BUILD_EXIT"
    echo "Timestamp (UTC): $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "========================================"
} >> "$BUILD_LOG"

exit $BUILD_EXIT
