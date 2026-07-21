#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$RUN_DIR/../../.." && pwd)"

: "${TEST_PROFILE:=rl_fixedstand_fast}"
: "${GUI:=false}"
: "${POLICY_PATH:=/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_stair.pt}"
: "${CAPTURE_ROOT:=$RUN_DIR/raw/runtime/rl_fast_smoke}"
: "${CASE_FILTER:=}"
: "${TRANSITION_GRACE:=1.0}"
: "${EVALUATION_DURATION:=4.0}"
: "${CAPTURE_WALL_TIMEOUT:=240}"
: "${START_PORT:=12111}"

mkdir -p "$CAPTURE_ROOT" "$RUN_DIR/metrics"

if [[ "$TEST_PROFILE" != "rl_fixedstand_fast" && "$TEST_PROFILE" != "rl_fast_smoke" ]]; then
  echo "ERROR: unsupported TEST_PROFILE=$TEST_PROFILE" >&2
  exit 2
fi

unset PYTHONHOME
unset PYTHONPATH
export PYTHONNOUSERSITE=1
export PATH="/usr/local/cuda-11.8/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"
export LD_LIBRARY_PATH="/home/zzf/third_party/libtorch-2.0.1-cu118-cxx11-abi/lib:/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH:-}"
hash -r

source /opt/ros/noetic/setup.bash
source "$REPO_ROOT/devel/setup.bash"

SUMMARY_JSON="$CAPTURE_ROOT/aggregate.json"
SUMMARY_MD="$CAPTURE_ROOT/summary.md"
RESULTS_JSONL="$CAPTURE_ROOT/results.jsonl"
: > "$RESULTS_JSONL"

case_rows=(
  "F0_native_fixedstand|native|earth|fixedstand|${START_PORT}|$((START_PORT + 34))"
  "F1_competition_fixedstand|auto|competition|fixedstand|$((START_PORT + 100))|$((START_PORT + 134))"
  "F2_earth_fixedstand|auto|earth|fixedstand|$((START_PORT + 200))|$((START_PORT + 234))"
)

log() {
  echo "[run_rl_fast_smoke] $*"
}

port_busy() {
  local port="$1"
  ss -ltn "( sport = :$port )" 2>/dev/null | grep -q ":$port"
}

snapshot_processes() {
  local dest="$1"
  {
    echo "timestamp=$(date -Iseconds)"
    pgrep -af 'roscore|rosmaster|roslaunch|gzserver|gzclient|junior_ctrl|state_from_gazebo' || true
    for pid in $(pgrep -f 'roscore|rosmaster|roslaunch|gzserver|gzclient|junior_ctrl|state_from_gazebo' || true); do
      echo "--- pid=$pid ---"
      ps -o pid,ppid,pgid,etime,cmd -p "$pid" 2>/dev/null || true
      readlink -f "/proc/$pid/exe" 2>/dev/null || true
      tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -E '^(ROS_|GAZEBO_|CMAKE_PREFIX_PATH|ROS_PACKAGE_PATH|LD_LIBRARY_PATH)=' || true
    done
  } > "$dest" 2>&1
}

append_result() {
  local json_file="$1"
  python3 - "$json_file" "$RESULTS_JSONL" <<'PYEOF'
import json
import sys
with open(sys.argv[1]) as f:
    data = json.load(f)
with open(sys.argv[2], "a") as f:
    f.write(json.dumps(data, sort_keys=True) + "\n")
PYEOF
}

ensure_native_controllers() {
  local out="$1"
  local ready_topic="/a1_gazebo/FR_hip_controller/state"
  for _ in $(seq 1 60); do
    if rostopic type "$ready_topic" 2>/dev/null | grep -q unitree_legged_msgs/MotorState; then
      return 0
    fi
    sleep 1
  done

  log "native controllers not ready; retrying controller_manager spawner"
  python3 /opt/ros/noetic/lib/controller_manager/spawner \
    joint_state_controller \
    FL_hip_controller FL_thigh_controller FL_calf_controller \
    FR_hip_controller FR_thigh_controller FR_calf_controller \
    RL_hip_controller RL_thigh_controller RL_calf_controller \
    RR_hip_controller RR_thigh_controller RR_calf_controller \
    __ns:=/a1_gazebo > "$out/controller_spawner_retry.log" 2>&1 || true

  for _ in $(seq 1 60); do
    if rostopic type "$ready_topic" 2>/dev/null | grep -q unitree_legged_msgs/MotorState; then
      return 0
    fi
    sleep 1
  done
  return 1
}

kill_group() {
  local pid="$1"
  [[ -z "$pid" ]] && return 0
  local pgid
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  [[ -z "$pgid" ]] && return 0
  kill -TERM -- "-$pgid" 2>/dev/null || true
}

kill_group_hard() {
  local pid="$1"
  [[ -z "$pid" ]] && return 0
  local pgid
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  [[ -z "$pgid" ]] && return 0
  kill -KILL -- "-$pgid" 2>/dev/null || true
}

cleanup_case() {
  local out="$1"
  local ros_port="$2"
  local gazebo_port="$3"
  local auto_pid="${4:-}"
  local roslaunch_pid="${5:-}"
  local ctrl_pid="${6:-}"
  local roscore_pid="${7:-}"
  local tmux_prefix="${8:-}"

  set +e
  mkdir -p "$out"
  snapshot_processes "$out/processes_after.txt"
  if [[ -n "$tmux_prefix" ]]; then
    tmux list-sessions -F '#S' 2>/dev/null | awk -v p="$tmux_prefix" 'index($0,p)==1 {print $0}' | while read -r session; do
      tmux capture-pane -t "$session" -p -S -3000 > "$out/${session}.log" 2>/dev/null || true
      tmux kill-session -t "$session" 2>/dev/null || true
    done
  fi
  for pid in "$auto_pid" "$roslaunch_pid" "$ctrl_pid" "$roscore_pid"; do
    kill_group "$pid"
  done
  sleep 1
  for pid in "$auto_pid" "$roslaunch_pid" "$ctrl_pid" "$roscore_pid"; do
    kill_group_hard "$pid"
  done
  ps -eo pid,args | awk -v rp="$ros_port" -v gp="$gazebo_port" '
    ($0 ~ "rosmaster --core -p " rp || $0 ~ "roscore -p " rp || $0 ~ "GAZEBO_MASTER_URI=http://127.0.0.1:" gp || $0 ~ "GAZEBO_MASTER_URI=http://localhost:" gp) && $0 !~ /awk/ {print $1}
  ' | while read -r pid; do
    kill -KILL "$pid" 2>/dev/null || true
  done
  ss -ltnp "( sport = :$gazebo_port )" 2>/dev/null \
    | sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' \
    | while read -r pid; do
      pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
      if [[ -n "$pgid" ]]; then
        kill -TERM -- "-$pgid" 2>/dev/null || true
        sleep 0.5
        kill -KILL -- "-$pgid" 2>/dev/null || true
      else
        kill -KILL "$pid" 2>/dev/null || true
      fi
    done
  set -e
}

write_skipped_case() {
  local case_id="$1"
  local out="$CAPTURE_ROOT/$case_id"
  mkdir -p "$out"
  cat > "$out/verdict.json" <<EOF
{
  "schema_version": 1,
  "case_id": "$case_id",
  "verdict": "NOT_RUN",
  "reason": "previous FixedStand gate did not pass"
}
EOF
  cp "$out/verdict.json" "$RUN_DIR/metrics/${case_id}.json"
  append_result "$out/verdict.json"
}

run_case() {
  local case_id="$1"
  local launch_mode="$2"
  local world_mode="$3"
  local case_kind="$4"
  local ros_port="$5"
  local gazebo_port="$6"
  local out="$CAPTURE_ROOT/$case_id"
  local auto_pid=""
  local roslaunch_pid=""
  local ctrl_pid=""
  local roscore_pid=""
  local tmux_prefix="simenv-rlfast-${case_id}"

  mkdir -p "$out"
  export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
  export GAZEBO_MASTER_URI="http://127.0.0.1:${gazebo_port}"

  if port_busy "$ros_port" || port_busy "$gazebo_port"; then
    cat > "$out/verdict.json" <<EOF
{
  "schema_version": 1,
  "case_id": "$case_id",
  "verdict": "FAIL_MASTER_MISMATCH",
  "reason": "requested ROS/Gazebo port already has a listener",
  "ros_port": $ros_port,
  "gazebo_port": $gazebo_port
}
EOF
    cp "$out/verdict.json" "$RUN_DIR/metrics/${case_id}.json"
    append_result "$out/verdict.json"
    return 1
  fi

  {
    date -Is
    echo "CASE_ID=$case_id"
    echo "CASE_KIND=$case_kind"
    echo "LAUNCH_MODE=$launch_mode"
    echo "WORLD_MODE_CASE=$world_mode"
    echo "ROS_MASTER_URI=$ROS_MASTER_URI"
    echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
    echo "TEST_PROFILE=$TEST_PROFILE"
    echo "POLICY_PATH=$POLICY_PATH"
    echo "TRANSITION_GRACE=$TRANSITION_GRACE"
    echo "EVALUATION_DURATION=$EVALUATION_DURATION"
    echo "branch=$(git -C "$REPO_ROOT" branch --show-current)"
    echo "head=$(git -C "$REPO_ROOT" rev-parse HEAD)"
    echo "repo_root=$REPO_ROOT"
    echo "unitree_guide=$(rospack find unitree_guide)"
    echo "unitree_gazebo=$(rospack find unitree_gazebo)"
    echo "unitree_legged_control=$(rospack find unitree_legged_control)"
    echo "junior_ctrl=$(readlink -f "$REPO_ROOT/devel/lib/unitree_guide/junior_ctrl" 2>/dev/null || true)"
    sha256sum "$REPO_ROOT/devel/lib/unitree_guide/junior_ctrl" 2>/dev/null || true
    sha256sum "$REPO_ROOT/src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world" 2>/dev/null || true
    sha256sum "$POLICY_PATH" 2>/dev/null || true
  } > "$out/environment.txt"

  log "$case_id: starting roscore on $ROS_MASTER_URI"
  setsid roscore -p "$ros_port" > "$out/roscore.log" 2>&1 &
  roscore_pid=$!
  for _ in $(seq 1 60); do
    rostopic list >/dev/null 2>&1 && break
    sleep 0.5
  done
  if ! rostopic list >/dev/null 2>&1; then
    echo "{\"schema_version\":1,\"case_id\":\"$case_id\",\"verdict\":\"FAIL_ROS_CLOCK_PUBLISH\",\"reason\":\"roscore did not become reachable\"}" > "$out/verdict.json"
    cp "$out/verdict.json" "$RUN_DIR/metrics/${case_id}.json"
    append_result "$out/verdict.json"
    cleanup_case "$out" "$ros_port" "$gazebo_port" "$auto_pid" "$roslaunch_pid" "$ctrl_pid" "$roscore_pid" "$tmux_prefix"
    return 1
  fi

  rosparam set /timing_diagnostics_enabled true
  rosparam set /timing_diagnostics_path "$out/controller_state.csv"

  case "$launch_mode" in
    native)
      log "$case_id: launching native unitree_gazebo normal.launch"
      setsid roslaunch unitree_gazebo normal.launch rname:=a1 gui:="$GUI" paused:=false use_sim_time:=true \
        > "$out/roslaunch.log" 2>&1 &
      roslaunch_pid=$!
      ;;
    auto)
      log "$case_id: launching auto.sh WORLD_MODE=$world_mode"
      (
        cd "$REPO_ROOT"
        WORLD_MODE="$world_mode" GUI="$GUI" PAUSED=true AUTO_UNPAUSE=1 START_CONTROLLER=1 \
          ENABLE_FAST_LIO2=0 ENABLE_RVIZ=0 ENABLE_SENSOR_DATA=0 ENABLE_POINTCLOUD_CONVERTER=0 \
          ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=0 START_BUILDING_CONTROL=0 \
          TERMINAL_BACKEND=tmux SKIP_GLOBAL_PROCESS_CLEANUP=1 TMUX_SESSION_PREFIX="$tmux_prefix" \
          ./auto.sh
      ) > "$out/auto.log" 2>&1 &
      auto_pid=$!
      ;;
    *)
      echo "Unknown launch_mode=$launch_mode" >&2
      return 2
      ;;
  esac

  for _ in $(seq 1 180); do
    if rostopic type /gazebo/model_states 2>/dev/null | grep -q gazebo_msgs/ModelStates; then
      break
    fi
    sleep 1
  done
  if ! rostopic type /gazebo/model_states 2>/dev/null | grep -q gazebo_msgs/ModelStates; then
    tail -n 160 "$out/auto.log" "$out/roslaunch.log" 2>/dev/null || true
    echo "{\"schema_version\":1,\"case_id\":\"$case_id\",\"verdict\":\"FAIL_GAZEBO_SIM_STALL\",\"reason\":\"/gazebo/model_states did not appear\"}" > "$out/verdict.json"
    cp "$out/verdict.json" "$RUN_DIR/metrics/${case_id}.json"
    append_result "$out/verdict.json"
    cleanup_case "$out" "$ros_port" "$gazebo_port" "$auto_pid" "$roslaunch_pid" "$ctrl_pid" "$roscore_pid" "$tmux_prefix"
    return 1
  fi

  if [[ "$launch_mode" = "native" ]]; then
    if ! ensure_native_controllers "$out"; then
      echo "{\"schema_version\":1,\"case_id\":\"$case_id\",\"verdict\":\"FAIL_FSM_ENTRY\",\"reason\":\"native controller state topics did not appear\"}" > "$out/verdict.json"
      cp "$out/verdict.json" "$RUN_DIR/metrics/${case_id}.json"
      append_result "$out/verdict.json"
      cleanup_case "$out" "$ros_port" "$gazebo_port" "$auto_pid" "$roslaunch_pid" "$ctrl_pid" "$roscore_pid" "$tmux_prefix"
      return 1
    fi
    log "$case_id: starting junior_ctrl"
    setsid "$REPO_ROOT/devel/lib/unitree_guide/junior_ctrl" > "$out/junior_ctrl.log" 2>&1 &
    ctrl_pid=$!
    sleep 2
  fi

  snapshot_processes "$out/processes_during.txt"

  set +e
  timeout --signal=TERM --kill-after=10s "$CAPTURE_WALL_TIMEOUT"s \
    /usr/bin/python3 "$RUN_DIR/scripts/rl_fast_live_capture.py" \
      --case-id "$case_id" \
      --case-kind "$case_kind" \
      --world-mode "$world_mode" \
      --output-dir "$out" \
      --timing-csv "$out/controller_state.csv" \
      --transition-grace "$TRANSITION_GRACE" \
      --evaluation-duration "$EVALUATION_DURATION" \
      --wall-timeout "$CAPTURE_WALL_TIMEOUT" \
      --policy-path "$POLICY_PATH" \
      2>&1 | tee "$out/capture.log"
  local capture_status="${PIPESTATUS[0]}"
  set -e

  cp "$out/metrics.json" "$RUN_DIR/metrics/${case_id}.json" 2>/dev/null || true
  if [[ -f "$out/verdict.json" ]]; then
    append_result "$out/verdict.json"
  fi
  cleanup_case "$out" "$ros_port" "$gazebo_port" "$auto_pid" "$roslaunch_pid" "$ctrl_pid" "$roscore_pid" "$tmux_prefix"
  return "$capture_status"
}

overall_status=0
previous_gate_pass=true
for row in "${case_rows[@]}"; do
  IFS='|' read -r case_id launch_mode world_mode case_kind ros_port gazebo_port <<< "$row"
  if [[ -n "$CASE_FILTER" && "$case_id" != "$CASE_FILTER" ]]; then
    continue
  fi
  if [[ "$previous_gate_pass" != "true" ]]; then
    log "$case_id: skipped because previous gate did not pass"
    write_skipped_case "$case_id"
    continue
  fi
  log "$case_id: running $launch_mode/$world_mode/$case_kind on ROS=$ros_port Gazebo=$gazebo_port"
  if run_case "$case_id" "$launch_mode" "$world_mode" "$case_kind" "$ros_port" "$gazebo_port"; then
    verdict="$(python3 - "$RUN_DIR/metrics/${case_id}.json" <<'PYEOF'
import json, sys
with open(sys.argv[1]) as f:
    print(json.load(f).get("verdict", "UNKNOWN"))
PYEOF
)"
    if [[ "$verdict" != "PASS" ]]; then
      previous_gate_pass=false
      overall_status=1
    fi
  else
    previous_gate_pass=false
    overall_status=1
  fi
done

PYTHONPATH="$RUN_DIR/scripts:${PYTHONPATH:-}" python3 - "$RESULTS_JSONL" "$SUMMARY_JSON" "$SUMMARY_MD" <<'PYEOF'
import json
import sys
from pathlib import Path
import rl_fast_metrics as m

jsonl = Path(sys.argv[1])
summary_json = Path(sys.argv[2])
summary_md = Path(sys.argv[3])
rows = []
if jsonl.exists():
    for line in jsonl.read_text().splitlines():
        if line.strip():
            rows.append(json.loads(line))
verdict = m.worst_verdict([r.get("verdict", "UNKNOWN") for r in rows]) if rows else "UNKNOWN"
summary = {"schema_version": 1, "verdict": verdict, "cases": rows}
summary_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
with summary_md.open("w") as f:
    f.write("# RL Fast Smoke Summary\n\n")
    f.write(f"**Verdict:** `{verdict}`\n\n")
    f.write("| Case | Verdict | Reason |\n")
    f.write("| --- | --- | --- |\n")
    for row in rows:
        f.write(f"| {row.get('case_id', 'unknown')} | `{row.get('verdict', 'UNKNOWN')}` | {row.get('reason', '')} |\n")
PYEOF

log "aggregate artifacts: $SUMMARY_JSON $SUMMARY_MD"
exit "$overall_status"
