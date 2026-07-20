#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
RUN_ROOT="$WORKSPACE/experiments/runs/0720_earth-rl-motion"
TRIAL_ID="${TRIAL_ID:?set TRIAL_ID, e.g. E0_fixedstand}"
TRIAL_STATE="${TRIAL_STATE:?set TRIAL_STATE to fixedstand or rl}"
TRIAL_DURATION="${TRIAL_DURATION:?set TRIAL_DURATION in sim seconds}"
COMMAND_VX="${COMMAND_VX:-0.0}"
COMMAND_VY="${COMMAND_VY:-0.0}"
COMMAND_YAW_RATE="${COMMAND_YAW_RATE:-0.0}"
STOP_DURATION="${STOP_DURATION:-0.0}"
OUT="$RUN_ROOT/results/$TRIAL_ID"
AUTO_PID=""
ROS_PORT="${ROS_PORT:-}"
if [ -z "$ROS_PORT" ]; then
  ROS_PORT="$(awk -v id="$TRIAL_ID" 'BEGIN {
    value = 0;
    for (i = 1; i <= length(id); i++) value += index("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789", substr(id, i, 1));
    printf "%d", 13000 + (value % 1000)
  }')"
fi
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"

cleanup() {
  set +e
  cp "$WORKSPACE/logs/competition_gazebo.log" "$OUT/gazebo.log" 2>/dev/null || true
  tmux capture-pane -t "simenv-earth-${TRIAL_ID}-junior_ctrl" -p -S -3000 \
    > "$OUT/controller.log" 2>/dev/null || \
    cp "$WORKSPACE/logs/junior_ctrl.log" "$OUT/controller.log" 2>/dev/null || true
  if [ -n "$AUTO_PID" ]; then
    auto_pgid="$(ps -o pgid= -p "$AUTO_PID" 2>/dev/null | tr -d ' ' || true)"
    if [ -n "$auto_pgid" ]; then
      kill -KILL -- "-$auto_pgid" 2>/dev/null || true
    fi
  fi
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
  tmux kill-session -t "simenv-earth-${TRIAL_ID}-junior_ctrl" 2>/dev/null || true
  timeout 5s rosnode kill -a >/dev/null 2>&1 || true
  master_pid="$(pgrep -f "rosmaster --core -p ${ROS_PORT}" | head -n 1 || true)"
  [ -z "$master_pid" ] || kill -KILL "$master_pid" 2>/dev/null || true
  for pid in $(ps -eo pid,args | awk -v port="$ROS_PORT" -v world="$WORKSPACE/src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world" '
      ($0 ~ "rosmaster --core -p " port || index($0, world) > 0) && $0 !~ /awk/ {print $1}
    '); do
    kill -KILL "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

mkdir -p "$OUT"
source /opt/ros/noetic/setup.bash
if [ ! -f "$WORKSPACE/devel/setup.bash" ]; then
  echo "Missing $WORKSPACE/devel/setup.bash; build before running earth RL trials." >&2
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
  echo "TRIAL_ID=$TRIAL_ID"
  echo "TRIAL_STATE=$TRIAL_STATE"
  echo "COMMAND_VX=$COMMAND_VX"
  echo "COMMAND_VY=$COMMAND_VY"
  echo "COMMAND_YAW_RATE=$COMMAND_YAW_RATE"
  echo "TRIAL_DURATION=$TRIAL_DURATION"
  echo "STOP_DURATION=$STOP_DURATION"
  echo "ROS_MASTER_URI=$ROS_MASTER_URI"
  git -C "$WORKSPACE" status --short
  git -C "$WORKSPACE" branch --show-current
  git -C "$WORKSPACE" rev-parse HEAD
} > "$OUT/environment.txt"

cd "$WORKSPACE"
WORLD_MODE=earth \
GUI="${GUI:-false}" \
PAUSED="${PAUSED:-true}" \
AUTO_UNPAUSE="${AUTO_UNPAUSE:-1}" \
START_CONTROLLER=1 \
ENABLE_FAST_LIO2="${ENABLE_FAST_LIO2:-0}" \
ENABLE_RVIZ="${ENABLE_RVIZ:-0}" \
ENABLE_SENSOR_DATA="${ENABLE_SENSOR_DATA:-0}" \
ENABLE_POINTCLOUD_CONVERTER="${ENABLE_POINTCLOUD_CONVERTER:-0}" \
ENABLE_REFEREE_ODOM="${ENABLE_REFEREE_ODOM:-0}" \
ENABLE_GROUND_TRUTH="${ENABLE_GROUND_TRUTH:-0}" \
START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-0}" \
TERMINAL_BACKEND=tmux \
SKIP_GLOBAL_PROCESS_CLEANUP=1 \
TMUX_SESSION_PREFIX="simenv-earth-${TRIAL_ID}" \
UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.002}" \
setsid ./auto.sh > "$OUT/auto.log" 2>&1 &
AUTO_PID=$!

for _ in $(seq 1 180); do
  if rostopic type /gazebo/model_states 2>/dev/null | grep -q gazebo_msgs/ModelStates; then
    break
  fi
  sleep 1
done
if ! rostopic type /gazebo/model_states 2>/dev/null | grep -q gazebo_msgs/ModelStates; then
  tail -n 120 "$OUT/auto.log" >&2 || true
  exit 1
fi

set +e
timeout --signal=TERM --kill-after=10s 420s \
  /usr/bin/python3 "$RUN_ROOT/earth_rl_capture.py" \
  --trial-id "$TRIAL_ID" \
  --state "$TRIAL_STATE" \
  --vx "$COMMAND_VX" \
  --vy "$COMMAND_VY" \
  --yaw-rate "$COMMAND_YAW_RATE" \
  --duration "$TRIAL_DURATION" \
  --stop-duration "$STOP_DURATION" \
  --output-dir "$OUT" \
  --timing-csv "$OUT/controller_state.csv" \
  2>&1 | tee "$OUT/capture.log"
capture_status="${PIPESTATUS[0]}"
set -e

exit "$capture_status"
