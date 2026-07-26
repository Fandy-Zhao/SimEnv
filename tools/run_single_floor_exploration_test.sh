#!/usr/bin/env bash
set -euo pipefail
# ---------------------------------------------------------------------------
# run_single_floor_exploration_test.sh
#
# Automated single-floor exploration test with result recording.
#
# Usage:
#   ./tools/run_single_floor_exploration_test.sh [OPTIONS]
#
# Options:
#   --run-id <id>          Unique run identifier (default: auto-generated)
#   --output-dir <path>    Output directory (default: experiments/runs/<date>/<run_id>)
#   --max-sim-time <sec>   Maximum simulation time (default: 1800)
#   --finish-quiet-time <s> Quiet window for completion (default: 60)
#   --no-gui               Disable Gazebo GUI and RViz
#   --help                 Show this help
#
# Environment variables (override defaults):
#   EXPLORATION_RUN_ID
#   EXPLORATION_OUTPUT_DIR
#   EXPLORATION_MAX_SIM_TIME
#   EXPLORATION_FINISH_QUIET_TIME
# ---------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── Defaults ──
DEFAULT_DATE="$(date +%Y%m%d)"
RUN_ID="${EXPLORATION_RUN_ID:-${DEFAULT_DATE}_run_$(date +%H%M%S)}"
OUTPUT_DIR="${EXPLORATION_OUTPUT_DIR:-$REPO_ROOT/experiments/runs/${DEFAULT_DATE}_single_floor_exploration/$RUN_ID}"
MAX_SIM_TIME="${EXPLORATION_MAX_SIM_TIME:-1800}"
FINISH_QUIET_TIME="${EXPLORATION_FINISH_QUIET_TIME:-60}"
GUI_MODE="true"

# ── Parse arguments ──
while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id)
      RUN_ID="$2"; shift 2 ;;
    --output-dir)
      OUTPUT_DIR="$2"; shift 2 ;;
    --max-sim-time)
      MAX_SIM_TIME="$2"; shift 2 ;;
    --finish-quiet-time)
      FINISH_QUIET_TIME="$2"; shift 2 ;;
    --no-gui)
      GUI_MODE="false"; shift ;;
    --help|-h)
      head -30 "$0"
      exit 0 ;;
    *)
      echo "ERROR: Unknown option: $1" >&2
      echo "Use --help for usage." >&2
      exit 1 ;;
  esac
done

# ── Validate ──
if [ -z "$RUN_ID" ]; then
  echo "ERROR: RUN_ID is empty." >&2
  exit 1
fi

# Prevent overwriting existing results
if [ -d "$OUTPUT_DIR" ] && [ -n "$(ls -A "$OUTPUT_DIR" 2>/dev/null)" ]; then
  echo "ERROR: Output directory already exists and is non-empty: $OUTPUT_DIR" >&2
  echo "  Use --run-id to specify a different run ID." >&2
  exit 1
fi

# ── Check workspace state ──
cd "$REPO_ROOT"

echo "============================================"
echo " Single-Floor Exploration Test"
echo "============================================"

# Git info
if command -v git >/dev/null 2>&1 && git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  GIT_BRANCH="$(git branch --show-current)"
  GIT_HEAD="$(git rev-parse --short HEAD 2>/dev/null || echo UNKNOWN)"
  echo "  Branch:    $GIT_BRANCH"
  echo "  HEAD:      $GIT_HEAD"
  if [ -n "$(git status --porcelain 2>/dev/null)" ]; then
    echo "  WARNING: Working tree is NOT clean."
  fi
else
  echo "  WARNING: Not in a git repository."
  GIT_BRANCH="UNKNOWN"
  GIT_HEAD="UNKNOWN"
fi

echo "  Run ID:    $RUN_ID"
echo "  Output:    $OUTPUT_DIR"
echo "  Max sim:   $MAX_SIM_TIME s"
echo "  Quiet:     $FINISH_QUIET_TIME s"
echo "  GUI:       $GUI_MODE"
echo ""

# ── Check build ──
if [ ! -x "$REPO_ROOT/devel/lib/unitree_guide/junior_ctrl" ]; then
  echo "ERROR: junior_ctrl not built. Run: $REPO_ROOT/tools/build_with_venv.sh" >&2
  exit 1
fi
if [ ! -f "$REPO_ROOT/devel/setup.bash" ]; then
  echo "ERROR: Workspace not built. devel/setup.bash missing." >&2
  exit 1
fi
echo "  Build check: PASS"

# ── Create output directory ──
mkdir -p "$OUTPUT_DIR"/{config,map,route,goals,plots,timing,logs,bags}
echo "  Created output directory: $OUTPUT_DIR"

# ── Write test config ──
cat > "$OUTPUT_DIR/config/test_config.txt" << EOF
run_id=$RUN_ID
output_dir=$OUTPUT_DIR
max_sim_time=$MAX_SIM_TIME
finish_quiet_time=$FINISH_QUIET_TIME
gui_mode=$GUI_MODE
git_branch=$GIT_BRANCH
git_head=$GIT_HEAD
start_wall_time=$(date -Iseconds)
EOF

# ── Launch auto.sh with recording enabled ──
echo ""
echo "=== Launching auto.sh ==="
echo ""

export GUI="$GUI_MODE"
export PAUSED="true"
export AUTO_UNPAUSE="true"
export ENABLE_FAST_LIO2="true"
export ENABLE_NAVIGATION="true"
export NAV_MODE="dsv_falco"
export NAV_AUTO_TROTTING="true"
export NAV_AUTO_ENABLE="true"
export NAV_AUTO_START_EXPLORATION="true"
export ENABLE_RVIZ="$GUI_MODE"

# Recording parameters
export ENABLE_EXPLORATION_RECORDING="true"
export EXPLORATION_RUN_ID="$RUN_ID"
export EXPLORATION_OUTPUT_DIR="$OUTPUT_DIR"
export EXPLORATION_MAX_SIM_TIME="$MAX_SIM_TIME"
export EXPLORATION_FINISH_QUIET_TIME="$FINISH_QUIET_TIME"

echo "Configuration exported. Launching auto.sh..."

# Run auto.sh in background
cd "$REPO_ROOT"
bash "$REPO_ROOT/auto.sh" > "$OUTPUT_DIR/logs/auto_stdout.log" 2>&1 &
AUTO_PID=$!
echo "$AUTO_PID" > "$OUTPUT_DIR/logs/auto.pid"

echo "auto.sh launched (pid=$AUTO_PID)."
echo "Waiting for exploration to start..."

# ── Wait for exploration start ──
START_TIMEOUT=300
START_COUNT=0
echo -n "  Waiting for exploration start signal..."
while [ $START_COUNT -lt $START_TIMEOUT ]; do
  if rostopic echo /navigation/start_exploring -n 1 2>/dev/null | grep -q "data: True"; then
    echo " OK"
    break
  fi
  sleep 2
  START_COUNT=$((START_COUNT + 2))
  echo -n "."
done

if [ $START_COUNT -ge $START_TIMEOUT ]; then
  echo " TIMEOUT"
  echo "WARNING: Exploration start signal not received within ${START_TIMEOUT}s."
  echo "  Continuing to monitor anyway..."
fi

# ── Monitor exploration progress ──
echo ""
echo "=== Monitoring Exploration ==="
echo ""

MAX_WAIT=$((MAX_SIM_TIME * 2))  # wall-clock max (assume RTF >= 0.5)
MONITOR_COUNT=0
while [ $MONITOR_COUNT -lt $MAX_WAIT ]; do
  # Check if recorder has finished
  if [ -f "$OUTPUT_DIR/logs/recorder.pid" ]; then
    RECORDER_PID="$(cat "$OUTPUT_DIR/logs/recorder.pid" 2>/dev/null || true)"
    if [ -n "$RECORDER_PID" ] && ! kill -0 "$RECORDER_PID" 2>/dev/null; then
      echo ""
      echo "Recorder process ($RECORDER_PID) has exited."
      break
    fi
  fi

  # Check if auto.sh is still running
  if ! kill -0 "$AUTO_PID" 2>/dev/null; then
    echo ""
    echo "auto.sh process ($AUTO_PID) has exited."
    break
  fi

  # Check for completion marker
  if [ -f "$OUTPUT_DIR/summary.md" ]; then
    echo ""
    echo "summary.md found — exploration results saved."
    break
  fi

  # Periodic status
  if [ $((MONITOR_COUNT % 30)) -eq 0 ] && [ $MONITOR_COUNT -gt 0 ]; then
    echo "  [$MONITOR_COUNT s] Waiting for exploration completion..."
    # Print current sim time if available
    SIM_TIME=$(rostopic echo /clock -n 1 2>/dev/null | grep "secs:" | head -1 | awk '{print $2}' || echo "?")
    echo "  Sim time: $SIM_TIME s"
  fi

  sleep 5
  MONITOR_COUNT=$((MONITOR_COUNT + 5))
done

if [ $MONITOR_COUNT -ge $MAX_WAIT ]; then
  echo ""
  echo "=== TIMEOUT ==="
  echo "Maximum wall-clock wait ($MAX_WAIT s) exceeded."
fi

# ── Post-run: ensure clean stop ──
echo ""
echo "=== Post-run Cleanup ==="

# Send zero velocity
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0, y: 0, z: 0} angular: {x: 0, y: 0, z: 0}" -1 2>/dev/null || true

# Stop navigation
rostopic pub /navigation/enabled std_msgs/Bool "data: false" -1 2>/dev/null || true

# Give time for final data flush
sleep 5

# ── Kill remaining processes ──
echo "Stopping simulation processes..."
pkill -9 -f "exploration_result_recorder" 2>/dev/null || true
pkill -9 -f "terrain_to_2d_map" 2>/dev/null || true

# ── Run validation ──
echo ""
echo "=== Result Validation ==="

VALIDATION_SCRIPT="$REPO_ROOT/tools/validate_single_floor_exploration_result.py"
if [ -f "$VALIDATION_SCRIPT" ]; then
  /usr/bin/python3 "$VALIDATION_SCRIPT" "$OUTPUT_DIR" 2>&1 | tee "$OUTPUT_DIR/logs/validation.log" || true
else
  echo "WARNING: Validation script not found: $VALIDATION_SCRIPT"
fi

# ── Final report ──
echo ""
echo "============================================"
echo " Test Complete"
echo "============================================"
echo "  Run ID:    $RUN_ID"
echo "  Output:    $OUTPUT_DIR"
echo ""
echo "  Result files:"
find "$OUTPUT_DIR" -type f | sort | while read -r f; do
  rel="$(realpath --relative-to="$OUTPUT_DIR" "$f" 2>/dev/null || echo "$f")"
  size="$(stat -c%s "$f" 2>/dev/null || echo 0)"
  echo "    $rel  ($size bytes)"
done

echo ""
echo "  To view results:"
echo "    ls -la $OUTPUT_DIR"
echo "    cat $OUTPUT_DIR/summary.md"
echo "    eog $OUTPUT_DIR/plots/map_route_goals.png"

# Cleanup auto.sh if still running
if kill -0 "$AUTO_PID" 2>/dev/null; then
  echo ""
  echo "Stopping auto.sh (pid=$AUTO_PID)..."
  kill -TERM "$AUTO_PID" 2>/dev/null || true
  sleep 2
  kill -9 "$AUTO_PID" 2>/dev/null || true
fi

echo ""
echo "Done."
