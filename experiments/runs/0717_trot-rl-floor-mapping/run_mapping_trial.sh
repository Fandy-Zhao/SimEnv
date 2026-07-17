#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODE="${MODE:?set MODE=trotting or MODE=rl}"
OUT="$WORKSPACE/experiments/runs/0717_trot-rl-floor-mapping/$MODE"
mkdir -p "$OUT"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

AUTO_PID=""
owned_pids=()
collect_descendants() {
  local parent="$1" child
  while read -r child; do
    [ -n "$child" ] || continue
    collect_descendants "$child"
  done < <(pgrep -P "$parent" 2>/dev/null || true)
  owned_pids+=("$parent")
}
cleanup() {
  if [ -n "$AUTO_PID" ] && kill -0 "$AUTO_PID" 2>/dev/null; then
    # auto.sh normally performs broad pkill cleanup. This isolated experiment
    # owns the auto.sh process tree, so terminate only that tree and its tmux.
    owned_pids=()
    collect_descendants "$AUTO_PID"
    if [ "${#owned_pids[@]}" -gt 0 ]; then
      kill -KILL "${owned_pids[@]}" 2>/dev/null || true
    fi
    wait "$AUTO_PID" 2>/dev/null || true
  fi
  tmux kill-session -t "simenv-0717-$MODE-junior_ctrl" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cd "$WORKSPACE"
FLOOR_COUNT=1 SEED=77 GUI=false ENABLE_RVIZ=false PAUSED=true AUTO_UNPAUSE=1 \
  START_CONTROLLER=1 ENABLE_FAST_LIO2=1 TERMINAL_BACKEND=tmux \
  SKIP_GLOBAL_PROCESS_CLEANUP=1 TMUX_SESSION_PREFIX="simenv-0717-$MODE" \
  setsid ./auto.sh > "$OUT/auto.log" 2>&1 &
AUTO_PID=$!

for _ in $(seq 1 240); do
  if ! kill -0 "$AUTO_PID" 2>/dev/null; then
    tail -n 100 "$OUT/auto.log" >&2
    exit 1
  fi
  if rostopic type /cloud_registered 2>/dev/null | grep -q sensor_msgs/PointCloud2; then
    break
  fi
  sleep 1
done

timeout --signal=TERM --kill-after=10s 240s \
  /usr/bin/python3 "$WORKSPACE/experiments/runs/0717_trot-rl-floor-mapping/capture_mapping_trial.py" \
  --mode "$MODE" --output-dir "$OUT" | tee "$OUT/capture.log"
cp "$WORKSPACE/logs/junior_ctrl.log" "$OUT/junior_ctrl.log" 2>/dev/null || true
cp "$WORKSPACE/logs/fast_lio2.log" "$OUT/fast_lio2.log" 2>/dev/null || true
