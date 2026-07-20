#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
RUN_ROOT="$WORKSPACE/experiments/runs/0718_g2_trotting_motion_baseline"
PROBE_MODE="${PROBE_MODE:?set PROBE_MODE: p0_fixedstand, p1_trotting_zero, or p2_trotting_vx}"
PROBE_ID="${PROBE_ID:?set PROBE_ID, e.g. p0_fixedstand_run_01}"
COMMAND_VX="${COMMAND_VX:-0.0}"
OUT="$RUN_ROOT/fast_exit/$PROBE_ID"
ROS_PORT="${ROS_PORT:-}"
if [ -z "$ROS_PORT" ]; then
  ROS_PORT="$(awk -v id="$PROBE_ID" 'BEGIN {
    gsub(/[^0-9]/, "", id); suffix = (id == "" ? 0 : int(id)) % 100;
    printf "%d", 13100 + suffix
  }')"
fi
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
mkdir -p "$OUT"

cleanup() {
  set +e
  for pid_file in "$WORKSPACE/logs/competition_gazebo.pid" "$WORKSPACE/logs/building_control.pid"; do
    if [ -r "$pid_file" ]; then
      read -r owned_pid < "$pid_file" || true
      owned_pgid="$(ps -o pgid= -p "$owned_pid" 2>/dev/null | tr -d ' ' || true)"
      if [ -n "$owned_pgid" ]; then
        kill -TERM -- "-$owned_pgid" 2>/dev/null || true
        sleep 1
        kill -KILL -- "-$owned_pgid" 2>/dev/null || true
      fi
    fi
  done
  tmux kill-session -t "simenv-g2fast-${PROBE_ID}-junior_ctrl" 2>/dev/null || true
  timeout 5s rosnode kill -a >/dev/null 2>&1 || true
  master_pid="$(pgrep -f "rosmaster --core -p ${ROS_PORT}" | head -n 1 || true)"
  [ -z "$master_pid" ] || kill -KILL "$master_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

source /opt/ros/noetic/setup.bash
if [ ! -f "$WORKSPACE/devel/setup.bash" ]; then
  echo "Missing $WORKSPACE/devel/setup.bash; build before running G2 fast-exit probes." >&2
  exit 2
fi
source "$WORKSPACE/devel/setup.bash"

if rostopic list >/dev/null 2>&1; then
  echo "Refusing to reuse an existing ROS master at $ROS_MASTER_URI" >&2
  exit 2
fi

roscore -p "$ROS_PORT" > "$OUT/roscore.log" 2>&1 &
for _ in $(seq 1 60); do
  if rostopic list >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done
if ! rostopic list >/dev/null 2>&1; then
  echo "Private roscore failed to start at $ROS_MASTER_URI" >&2
  exit 2
fi
rosparam set /timing_diagnostics_enabled true
rosparam set /timing_diagnostics_path "$OUT/controller_state.csv"

{
  date -Is
  echo "WORKSPACE=$WORKSPACE"
  echo "PROBE_ID=$PROBE_ID"
  echo "PROBE_MODE=$PROBE_MODE"
  echo "COMMAND_VX=$COMMAND_VX"
  echo "ROS_MASTER_URI=$ROS_MASTER_URI"
  env | grep -E '^(FLOOR_COUNT|SEED|GUI|UNITREE_CTRL_DT|GAZEBO_PHYSICS|ROBOT_|ROS_MASTER_URI)=' | sort || true
} > "$OUT/environment.txt"
{
  git -C "$WORKSPACE" status --short
  git -C "$WORKSPACE" branch --show-current
  git -C "$WORKSPACE" rev-parse HEAD
} > "$OUT/git.txt"

cat > "$OUT/manifest.json" <<EOF
{
  "schema_version": 1,
  "probe_id": "$PROBE_ID",
  "probe_mode": "$PROBE_MODE",
  "command_vx": $COMMAND_VX,
  "ros_master_uri": "$ROS_MASTER_URI",
  "workspace": "$WORKSPACE"
}
EOF

cd "$WORKSPACE"
FLOOR_COUNT="${FLOOR_COUNT:-1}" \
SEED="${SEED:-77}" \
GUI="${GUI:-false}" \
ENABLE_RVIZ="${ENABLE_RVIZ:-false}" \
PAUSED="${PAUSED:-true}" \
AUTO_UNPAUSE="${AUTO_UNPAUSE:-1}" \
START_CONTROLLER=1 \
ENABLE_FAST_LIO2=0 \
ENABLE_SENSOR_DATA=0 \
ENABLE_POINTCLOUD_CONVERTER=0 \
ENABLE_FOOT_FORCE_VISUAL=1 \
START_BUILDING_CONTROL=0 \
TERMINAL_BACKEND=tmux \
SKIP_GLOBAL_PROCESS_CLEANUP=1 \
TMUX_SESSION_PREFIX="simenv-g2fast-${PROBE_ID}" \
UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.002}" \
setsid ./auto.sh > "$OUT/auto.log" 2>&1 &

for _ in $(seq 1 180); do
  if rostopic type /gazebo/model_states 2>/dev/null | grep -q gazebo_msgs/ModelStates; then
    break
  fi
  sleep 1
done
if ! rostopic type /gazebo/model_states 2>/dev/null | grep -q gazebo_msgs/ModelStates; then
  tail -n 100 "$OUT/auto.log" >&2
  exit 1
fi

rosparam dump "$OUT/rosparams.yaml" || true

set +e
timeout --signal=TERM --kill-after=10s 360s \
  /usr/bin/python3 "$RUN_ROOT/g2_fast_exit_probe.py" \
  --probe-id "$PROBE_ID" \
  --probe-mode "$PROBE_MODE" \
  --command-vx "$COMMAND_VX" \
  --output-dir "$OUT" \
  --timing-csv "$OUT/controller_state.csv" \
  2>&1 | tee "$OUT/probe.log"
probe_status="${PIPESTATUS[0]}"
set -e

cp "$WORKSPACE/logs/competition_gazebo.log" "$OUT/gazebo.log" 2>/dev/null || true
tmux capture-pane -t "simenv-g2fast-${PROBE_ID}-junior_ctrl" -p -S -3000 \
  > "$OUT/controller.log" 2>/dev/null || \
  cp "$WORKSPACE/logs/junior_ctrl.log" "$OUT/controller.log" 2>/dev/null || true
sha256sum \
  "$WORKSPACE/generated_building/competition_scene.world" \
  "$WORKSPACE/src/unitree_guide/unitree_ros/robots/a1_description/urdf/a1.urdf" \
  "$WORKSPACE/devel/lib/unitree_guide/junior_ctrl" \
  "$WORKSPACE/devel/lib/libunitreeFootContactPlugin.so" \
  > "$OUT/binary_hashes.txt" 2>/dev/null || true

exit "$probe_status"
