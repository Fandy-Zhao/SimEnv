#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODE="${MODE:?MODE=trotting or rl}"
SPEED="${SPEED:?SPEED=0.1, 0.5, or 1.0}"
TAG="${MODE}_$(printf '%03d' "$(awk "BEGIN {print $SPEED * 100}")")"
OUT="$WORKSPACE/experiments/runs/0717_trot-rl-speed-profile/raw/$TAG"
mkdir -p "$OUT"
case "$TAG" in
  trotting_010) ROS_PORT=11410 ;;
  trotting_050) ROS_PORT=11450 ;;
  trotting_100) ROS_PORT=11500 ;;
  rl_010) ROS_PORT=11610 ;;
  rl_050) ROS_PORT=11650 ;;
  rl_100) ROS_PORT=11700 ;;
  *) echo "unsupported trial tag: $TAG" >&2; exit 2 ;;
esac
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
: "${SIMENV_BINARY_DEVEL:?set SIMENV_BINARY_DEVEL to a verified Torch-enabled devel directory}"

AUTO_PID=""
cleanup() {
  # auto.sh returns after startup; its roslaunch PID is the owned process to
  # stop. Do not use global pkill: another workspace may have a ROS session.
  for pid_file in "$WORKSPACE/logs/competition_gazebo.pid" "$WORKSPACE/logs/building_control.pid"; do
    if [ -r "$pid_file" ]; then
      read -r owned_pid < "$pid_file" || true
      owned_pgid="$(ps -o pgid= -p "$owned_pid" 2>/dev/null | tr -d ' ' || true)"
      if [ -n "$owned_pgid" ]; then
        kill -TERM -- -"$owned_pgid" 2>/dev/null || true
        for _ in $(seq 1 20); do
          kill -0 "$owned_pid" 2>/dev/null || break
          sleep 0.25
        done
        kill -KILL -- -"$owned_pgid" 2>/dev/null || true
      fi
    fi
  done
  tmux kill-session -t "simenv-0717-speed-${TAG}-junior_ctrl" 2>/dev/null || true
  # Each run has its own master port, so these commands cannot affect a user
  # ROS session or a different test epoch.
  timeout 5s rosnode kill -a >/dev/null 2>&1 || true
  master_pid="$(pgrep -f "rosmaster --core -p ${ROS_PORT}" | head -n 1 || true)"
  [ -z "$master_pid" ] || kill -KILL "$master_pid" 2>/dev/null || true
  # Do not start the next epoch until this epoch's private master is gone.
  for _ in $(seq 1 40); do
    pgrep -f "rosmaster --core -p ${ROS_PORT}" >/dev/null || break
    sleep 0.25
  done
}
trap cleanup EXIT INT TERM

# The isolated worktree intentionally reuses verified binaries while exposing
# this worktree as the only catkin source path. The .catkin marker lets
# roslaunch locate state_from_gazebo in the linked lib directory.
if [ -L "$WORKSPACE/devel" ]; then
  rm "$WORKSPACE/devel"
fi
if [ -d "$WORKSPACE/devel" ]; then
  rm -f "$WORKSPACE/devel/setup.bash" "$WORKSPACE/devel/lib"
  rm -f "$WORKSPACE/devel/.catkin"
  rmdir "$WORKSPACE/devel"
fi
mkdir "$WORKSPACE/devel"
ln -s "$SIMENV_BINARY_DEVEL/lib" "$WORKSPACE/devel/lib"
ln -s "$WORKSPACE/experiments/runs/0717_trot-rl-speed-profile/runtime_setup.bash" "$WORKSPACE/devel/setup.bash"
ln -s "$WORKSPACE/experiments/runs/0717_trot-rl-speed-profile/catkin_workspace_marker" "$WORKSPACE/devel/.catkin"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

if rostopic list >/dev/null 2>&1; then
  echo "refusing to reuse an existing ROS master at $ROS_MASTER_URI" >&2
  exit 1
fi

cd "$WORKSPACE"
FLOOR_COUNT=1 SEED=77 GUI=false ENABLE_RVIZ=false PAUSED=true AUTO_UNPAUSE=1 \
  START_CONTROLLER=1 ENABLE_FAST_LIO2=0 TERMINAL_BACKEND=tmux \
  SKIP_GLOBAL_PROCESS_CLEANUP=1 TMUX_SESSION_PREFIX="simenv-0717-speed-${TAG}" \
  setsid ./auto.sh > "$OUT/auto.log" 2>&1 &
AUTO_PID=$!

# auto.sh deliberately returns after it has launched Gazebo and the controller.
# It is therefore not a lifetime owner of the asynchronous roslaunch process.
# Topic readiness, on this epoch's private ROS master, is the startup contract.
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

timeout --signal=TERM --kill-after=10s 240s \
  /usr/bin/python3 "$WORKSPACE/experiments/runs/0717_trot-rl-speed-profile/speed_profile_trial.py" \
  --mode "$MODE" --speed "$SPEED" --output-dir "$OUT" --wall-timeout 10 | tee "$OUT/capture.log"
cp "$WORKSPACE/logs/junior_ctrl.log" "$OUT/junior_ctrl.log" 2>/dev/null || true
