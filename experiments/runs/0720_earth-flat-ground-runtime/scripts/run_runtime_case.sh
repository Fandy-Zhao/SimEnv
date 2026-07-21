#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
RUN_ROOT="$WORKSPACE/experiments/runs/0720_earth-flat-ground-runtime"
CASE_ID="${CASE_ID:?set CASE_ID}"
CASE_KIND="${CASE_KIND:?set CASE_KIND: g0, g1, fixedstand, rl}"
WORLD_MODE_CASE="${WORLD_MODE_CASE:-earth}"
DURATION="${DURATION:-3.0}"
STOP_DURATION="${STOP_DURATION:-0.0}"
COMMAND_VX="${COMMAND_VX:-0.0}"
COMMAND_VY="${COMMAND_VY:-0.0}"
COMMAND_YAW_RATE="${COMMAND_YAW_RATE:-0.0}"
ROS_PORT="${ROS_PORT:-}"
GAZEBO_PORT="${GAZEBO_PORT:-}"
OUT="$RUN_ROOT/raw/$CASE_ID"
AUTO_PID=""
ROSLAUNCH_PID=""

if [ -z "$ROS_PORT" ]; then
  ROS_PORT="$(awk -v id="$CASE_ID" 'BEGIN {v=0; for(i=1;i<=length(id);i++) v+=index("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_", substr(id,i,1)); printf "%d", 14000 + (v % 1000)}')"
fi
if [ -z "$GAZEBO_PORT" ]; then
  GAZEBO_PORT="$((ROS_PORT + 1000))"
fi
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_PORT}"

unset PYTHONHOME
unset PYTHONPATH
export PYTHONNOUSERSITE=1
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"
hash -r

cleanup() {
  set +e
  mkdir -p "$OUT"
  pgrep -af 'roscore|rosmaster|gzserver|gzclient|junior_ctrl|state_from_gazebo' > "$OUT/processes_after.txt" 2>/dev/null || true
  cp "$WORKSPACE/logs/competition_gazebo.log" "$OUT/gazebo.log" 2>/dev/null || true
  tmux capture-pane -t "simenv-runtime-${CASE_ID}-junior_ctrl" -p -S -3000 > "$OUT/controller.log" 2>/dev/null || true
  tmux kill-session -t "simenv-runtime-${CASE_ID}-junior_ctrl" 2>/dev/null || true
  for pid in "$AUTO_PID" "$ROSLAUNCH_PID"; do
    [ -z "$pid" ] && continue
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
    [ -z "$pgid" ] || kill -TERM -- "-$pgid" 2>/dev/null || true
  done
  sleep 1
  for pid in "$AUTO_PID" "$ROSLAUNCH_PID"; do
    [ -z "$pid" ] && continue
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
    [ -z "$pgid" ] || kill -KILL -- "-$pgid" 2>/dev/null || true
  done
  timeout 5s rosnode kill -a >/dev/null 2>&1 || true
  for pid in $(ps -eo pid,args | awk -v rp="$ROS_PORT" -v gp="$GAZEBO_PORT" '($0 ~ "rosmaster --core -p " rp || $0 ~ "gzserver" || $0 ~ "GAZEBO_MASTER_URI=http://127.0.0.1:" gp) && $0 !~ /awk/ {print $1}'); do
    kill -KILL "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

mkdir -p "$OUT" "$RUN_ROOT/environment" "$RUN_ROOT/metrics"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

if rostopic list >/dev/null 2>&1; then
  echo "Refusing to reuse existing ROS master at $ROS_MASTER_URI" >&2
  exit 2
fi

{
  date -Is
  echo "CASE_ID=$CASE_ID"
  echo "CASE_KIND=$CASE_KIND"
  echo "WORLD_MODE_CASE=$WORLD_MODE_CASE"
  echo "ROS_MASTER_URI=$ROS_MASTER_URI"
  echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
  echo "ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
  echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
  echo "COMMAND_VX=$COMMAND_VX COMMAND_VY=$COMMAND_VY COMMAND_YAW_RATE=$COMMAND_YAW_RATE"
  echo "DURATION=$DURATION STOP_DURATION=$STOP_DURATION"
  echo "branch=$(git -C "$WORKSPACE" branch --show-current)"
  echo "head=$(git -C "$WORKSPACE" rev-parse HEAD)"
  echo "unitree_guide=$(rospack find unitree_guide)"
  echo "unitree_gazebo=$(rospack find unitree_gazebo)"
  echo "roslaunch=$(command -v roslaunch)"
  echo "gzserver=$(command -v gzserver)"
  sha256sum "$WORKSPACE/devel/lib/unitree_guide/junior_ctrl" 2>/dev/null || true
  sha256sum "$WORKSPACE/devel/lib/unitree_guide/state_from_gazebo" 2>/dev/null || true
  sha256sum "$WORKSPACE/src/unitree_guide/logs/policy_act_inference_plane.pt" 2>/dev/null || true
  sha256sum "$WORKSPACE/src/unitree_guide/logs/policy_act_inference_stair.pt" 2>/dev/null || true
  sha256sum "$WORKSPACE/src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world" 2>/dev/null || true
  pgrep -af 'roscore|rosmaster|gzserver|gzclient|junior_ctrl|state_from_gazebo' || true
} > "$OUT/environment.txt"

roscore -p "$ROS_PORT" > "$OUT/roscore.log" 2>&1 &
for _ in $(seq 1 60); do
  rostopic list >/dev/null 2>&1 && break
  sleep 0.5
done
rostopic list >/dev/null 2>&1 || { echo "roscore failed" >&2; exit 2; }

rosparam set /timing_diagnostics_enabled true
rosparam set /timing_diagnostics_path "$OUT/controller_state.csv"

if [ "$CASE_KIND" = "g0" ]; then
  setsid roslaunch gazebo_ros empty_world.launch \
    world_name:="$WORKSPACE/src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world" \
    gui:=false paused:=false use_sim_time:=true \
    > "$OUT/roslaunch.log" 2>&1 &
  ROSLAUNCH_PID=$!
else
  cd "$WORKSPACE"
  WORLD_MODE="$WORLD_MODE_CASE" GUI=false PAUSED=true AUTO_UNPAUSE=1 START_CONTROLLER=1 \
    ENABLE_FAST_LIO2=0 ENABLE_RVIZ=0 ENABLE_SENSOR_DATA=0 ENABLE_POINTCLOUD_CONVERTER=0 \
    ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=0 START_BUILDING_CONTROL=0 \
    TERMINAL_BACKEND=tmux SKIP_GLOBAL_PROCESS_CLEANUP=1 TMUX_SESSION_PREFIX="simenv-runtime-${CASE_ID}" \
    setsid ./auto.sh > "$OUT/auto.log" 2>&1 &
  AUTO_PID=$!
fi

for _ in $(seq 1 180); do
  if rostopic type /gazebo/model_states 2>/dev/null | grep -q gazebo_msgs/ModelStates; then
    break
  fi
  sleep 1
done
rostopic type /gazebo/model_states 2>/dev/null | grep -q gazebo_msgs/ModelStates || {
  tail -n 120 "$OUT/auto.log" "$OUT/roslaunch.log" 2>/dev/null || true
  exit 1
}

set +e
timeout --signal=TERM --kill-after=10s 480s \
  /usr/bin/python3 "$RUN_ROOT/scripts/runtime_capture.py" \
  --case-id "$CASE_ID" \
  --case-kind "$CASE_KIND" \
  --world-mode "$WORLD_MODE_CASE" \
  --duration "$DURATION" \
  --stop-duration "$STOP_DURATION" \
  --vx "$COMMAND_VX" \
  --vy "$COMMAND_VY" \
  --yaw-rate "$COMMAND_YAW_RATE" \
  --output-dir "$OUT" \
  --timing-csv "$OUT/controller_state.csv" \
  2>&1 | tee "$OUT/capture.log"
capture_status="${PIPESTATUS[0]}"
set -e
cp "$OUT/metrics.json" "$RUN_ROOT/metrics/${CASE_ID}.json" 2>/dev/null || true
exit "$capture_status"
