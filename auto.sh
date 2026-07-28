#!/usr/bin/env bash
set -euo pipefail

# ---------------------------------------------------------------------------
# Prevent Conda / Python environments from contaminating ROS Noetic.
# ---------------------------------------------------------------------------
unset PYTHONHOME
unset PYTHONPATH
export PYTHONNOUSERSITE=1

# Ensure system executables take precedence over Conda.
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"

hash -r

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

as_ros_bool() {
  case "$1" in
    1|true|TRUE|True|yes|YES|on|ON) printf "true" ;;
    0|false|FALSE|False|no|NO|off|OFF) printf "false" ;;
    *) printf "%s" "$1" ;;
  esac
}

shell_quote() {
  printf "%q" "$1"
}

WORLD_MODE="${WORLD_MODE:-competition}"
if [ "$WORLD_MODE" != "competition" ] && [ "$WORLD_MODE" != "earth" ]; then
  echo "ERROR: WORLD_MODE must be 'competition' or 'earth', got '$WORLD_MODE'" >&2
  exit 1
fi

if [ "$WORLD_MODE" = "earth" ]; then
  START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-0}"
  ENABLE_FAST_LIO2="${ENABLE_FAST_LIO2:-0}"
  ENABLE_RVIZ="${ENABLE_RVIZ:-0}"
  ENABLE_SENSOR_DATA="${ENABLE_SENSOR_DATA:-0}"
  ENABLE_POINTCLOUD_CONVERTER="${ENABLE_POINTCLOUD_CONVERTER:-0}"
  ENABLE_REFEREE_ODOM="${ENABLE_REFEREE_ODOM:-0}"
  ENABLE_GROUND_TRUTH="${ENABLE_GROUND_TRUTH:-0}"
  WRITE_GENERATED_TRUTH_COPY="${WRITE_GENERATED_TRUTH_COPY:-0}"
  ROBOT_X="${ROBOT_X:-0.0}"
  ROBOT_Y="${ROBOT_Y:-0.0}"
  ROBOT_Z="${ROBOT_Z:-0.6}"
  ROBOT_YAW="${ROBOT_YAW:-0.0}"
fi

# ---------------------------------------------------------------------------
# Physics profile: selects the Gazebo physics parameter preset.
#
#   PHYSICS_PROFILE=normal    Balance of performance and contact fidelity.
#                             Earth A1 average RTF >= 0.8 with reliable contact.
#   PHYSICS_PROFILE=fidelity  Original high-resolution configuration (0.0002 s /
#                             5000 Hz / ODE 50).  Low RTF; for special verification.
#
# Explicit single-parameter overrides (GAZEBO_PHYSICS_MAX_STEP_SIZE, etc.)
# take precedence over the profile default.  Unknown profile values are rejected.
#
# Default scope:
#   WORLD_MODE=earth       -> PHYSICS_PROFILE=normal (0.001 / 1000 / 20)
#   WORLD_MODE=competition -> Legacy defaults (0.002 / 500 / 40)
#                             until competition-mode regression is completed.
#   Explicit PHYSICS_PROFILE or GAZEBO_PHYSICS_* overrides always take effect.
# ---------------------------------------------------------------------------

# Detect whether the user explicitly configured physics (profile or individual
# parameters).  When nothing is set, competition mode preserves its pre-profile
# defaults.
_PHYSICS_USER_CONFIGURED=0
if [ -n "${PHYSICS_PROFILE+x}" ]; then _PHYSICS_USER_CONFIGURED=1; fi
if [ -n "${GAZEBO_PHYSICS_MAX_STEP_SIZE+x}" ]; then _PHYSICS_USER_CONFIGURED=1; fi
if [ -n "${GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE+x}" ]; then _PHYSICS_USER_CONFIGURED=1; fi
if [ -n "${GAZEBO_PHYSICS_ODE_ITERS+x}" ]; then _PHYSICS_USER_CONFIGURED=1; fi

PHYSICS_PROFILE="${PHYSICS_PROFILE:-normal}"
case "$PHYSICS_PROFILE" in
  normal)
    _PHYSICS_PROFILE_DEFAULT_STEP="0.001"
    _PHYSICS_PROFILE_DEFAULT_RATE="1000"
    _PHYSICS_PROFILE_DEFAULT_ITERS="20"
    ;;
  fidelity)
    _PHYSICS_PROFILE_DEFAULT_STEP="0.0002"
    _PHYSICS_PROFILE_DEFAULT_RATE="5000"
    _PHYSICS_PROFILE_DEFAULT_ITERS="50"
    ;;
  *)
    echo "[ERROR] Unsupported PHYSICS_PROFILE='${PHYSICS_PROFILE}'" >&2
    echo "Allowed values: normal, fidelity" >&2
    exit 1
    ;;
esac

SEED="${SEED:-}"
FLOOR_COUNT="${FLOOR_COUNT:-1}"
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}"
BUILDING_WIDTH="${BUILDING_WIDTH:-20.0}"
BUILDING_LENGTH="${BUILDING_LENGTH:-36.0}"
DANGER_COUNT="${DANGER_COUNT:-3:6}"
DISTRACTOR_COUNT="${DISTRACTOR_COUNT:-4:8}"
GUI="${GUI:-false}"
PAUSED="${PAUSED:-true}"
AUTO_UNPAUSE="$(as_ros_bool "${AUTO_UNPAUSE:-1}")"
AUTO_UNPAUSE_DELAY="${AUTO_UNPAUSE_DELAY:-6}"
# Optional watchdog: when true, gazebo_final_unpause will continuously monitor
# /clock and re-unpause if stalled.  Default off for manual testing.
AUTO_UNPAUSE_GAZEBO="${AUTO_UNPAUSE_GAZEBO:-false}"
GAZEBO_CLOCK_STALE_TIMEOUT="${GAZEBO_CLOCK_STALE_TIMEOUT:-3.0}"
GAZEBO_UNPAUSE_MAX_RETRIES="${GAZEBO_UNPAUSE_MAX_RETRIES:-3}"
START_CONTROLLER="${START_CONTROLLER:-1}"
START_VIRTUAL_JOY="${START_VIRTUAL_JOY:-0}"
START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-1}"
ENABLE_SENSOR_DATA_DEFAULT="${ENABLE_SENSORS:-1}"
ENABLE_SENSOR_DATA="$(as_ros_bool "${ENABLE_SENSOR_DATA:-$ENABLE_SENSOR_DATA_DEFAULT}")"
ENABLE_REFEREE_ODOM="$(as_ros_bool "${ENABLE_REFEREE_ODOM:-1}")"
ENABLE_GROUND_TRUTH="$(as_ros_bool "${ENABLE_GROUND_TRUTH:-1}")"
ENABLE_FOOT_FORCE_VISUAL="$(as_ros_bool "${ENABLE_FOOT_FORCE_VISUAL:-0}")"
ENABLE_JOY_NODE="$(as_ros_bool "${ENABLE_JOY_NODE:-0}")"
ENABLE_POINTCLOUD_CONVERTER="$(as_ros_bool "${ENABLE_POINTCLOUD_CONVERTER:-0}")"
ENABLE_LIDAR_VISUALIZATION="$(as_ros_bool "${ENABLE_LIDAR_VISUALIZATION:-0}")"
POINTCLOUD_USE_GROUND_TRUTH_ODOM="$(as_ros_bool "${POINTCLOUD_USE_GROUND_TRUTH_ODOM:-1}")"
WRITE_GENERATED_TRUTH_COPY="$(as_ros_bool "${WRITE_GENERATED_TRUTH_COPY:-1}")"
ENABLE_FAST_LIO2="$(as_ros_bool "${ENABLE_FAST_LIO2:-1}")"
FAST_LIO2_DELAY="${FAST_LIO2_DELAY:-5}"
ENABLE_RVIZ="$(as_ros_bool "${ENABLE_RVIZ:-1}")"

# Navigation bringup — single-floor exploration with DSV + FALCO.
# Requires ENABLE_FAST_LIO2=1.
# NAV_MODE=falco:       FALCO-only (waypoint from external source)
# NAV_MODE=dsv_falco:   DSV generates exploration waypoints, FALCO executes
ENABLE_NAVIGATION="$(as_ros_bool "${ENABLE_NAVIGATION:-0}")"
NAV_MODE="${NAV_MODE:-falco}"
NAV_MAX_LINEAR_X="${NAV_MAX_LINEAR_X:-0.80}"
NAV_MAX_LINEAR_Y="${NAV_MAX_LINEAR_Y:-0.00}"
NAV_MAX_ANGULAR_Z="${NAV_MAX_ANGULAR_Z:-0.22}"
NAV_COMMAND_TIMEOUT="${NAV_COMMAND_TIMEOUT:-0.50}"
NAV_AUTO_TROTTING="$(as_ros_bool "${NAV_AUTO_TROTTING:-${AUTO_COMMAND_TROTTING:-0}}")"
NAV_AUTO_ENABLE="$(as_ros_bool "${NAV_AUTO_ENABLE:-${AUTO_ENABLE_NAVIGATION:-0}}")"
NAV_AUTO_START_EXPLORATION="$(as_ros_bool "${NAV_AUTO_START_EXPLORATION:-${AUTO_START_EXPLORATION:-0}}")"
NAV_WAIT_ODOM_TIMEOUT="${NAV_WAIT_ODOM_TIMEOUT:-60}"
NAV_WAIT_CLOUD_TIMEOUT="${NAV_WAIT_CLOUD_TIMEOUT:-60}"
NAV_WAIT_TERRAIN_TIMEOUT="${NAV_WAIT_TERRAIN_TIMEOUT:-60}"
ROS_MASTER_TIMEOUT="${ROS_MASTER_TIMEOUT:-30}"
GAZEBO_READY_TIMEOUT="${GAZEBO_READY_TIMEOUT:-90}"
ROBOT_READY_TIMEOUT="${ROBOT_READY_TIMEOUT:-60}"
CONTROLLER_READY_TIMEOUT="${CONTROLLER_READY_TIMEOUT:-45}"
SENSOR_READY_TIMEOUT="${SENSOR_READY_TIMEOUT:-60}"
FAST_LIO2_READY_TIMEOUT="${FAST_LIO2_READY_TIMEOUT:-120}"
NAV_SUPERVISOR_READY_TIMEOUT="${NAV_SUPERVISOR_READY_TIMEOUT:-30}"
NAVIGATION_READY_TIMEOUT="${NAVIGATION_READY_TIMEOUT:-120}"
STATE_TRANSITION_TIMEOUT="${STATE_TRANSITION_TIMEOUT:-20}"
RECORDER_READY_TIMEOUT="${RECORDER_READY_TIMEOUT:-30}"
STARTUP_POLL_INTERVAL="${STARTUP_POLL_INTERVAL:-0.25}"
TOPIC_MESSAGE_TIMEOUT="${TOPIC_MESSAGE_TIMEOUT:-12}"
# ---------------------------------------------------------------------------
# Exploration result recording — save map, route, goals, timing, and summary.
# Default: OFF.  Set ENABLE_EXPLORATION_RECORDING=1 to activate.
#
#   ENABLE_EXPLORATION_RECORDING=1
#   EXPLORATION_RUN_ID=run_01
#   EXPLORATION_OUTPUT_DIR=<workspace>/experiments/runs/<date>/<run_id>
#   EXPLORATION_MAX_SIM_TIME=1800
#   EXPLORATION_FINISH_QUIET_TIME=60
# ---------------------------------------------------------------------------
ENABLE_EXPLORATION_RECORDING="$(as_ros_bool "${ENABLE_EXPLORATION_RECORDING:-0}")"
EXPLORATION_RUN_ID="${EXPLORATION_RUN_ID:-default_run}"
EXPLORATION_OUTPUT_DIR="${EXPLORATION_OUTPUT_DIR:-$WORKSPACE_DIR/experiments/runs/$(date +%Y%m%d)_single_floor_exploration/$EXPLORATION_RUN_ID}"
EXPLORATION_MAX_SIM_TIME="${EXPLORATION_MAX_SIM_TIME:-1800}"
EXPLORATION_FINISH_QUIET_TIME="${EXPLORATION_FINISH_QUIET_TIME:-60}"
EXPLORATION_MAP_STABLE_WAIT="${EXPLORATION_MAP_STABLE_WAIT:-5}"
TIMING_DIAGNOSTICS_ENABLED="$(as_ros_bool "${TIMING_DIAGNOSTICS_ENABLED:-0}")"
TIMING_DIAGNOSTICS_PATH="${TIMING_DIAGNOSTICS_PATH:-$WORKSPACE_DIR/logs/unitree_timing.csv}"
TERMINAL_BACKEND="${TERMINAL_BACKEND:-tmux}"
TMUX_SESSION_PREFIX="${TMUX_SESSION_PREFIX:-simenv}"
SKIP_GLOBAL_PROCESS_CLEANUP="$(as_ros_bool "${SKIP_GLOBAL_PROCESS_CLEANUP:-0}")"
UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.002}"
# Physics parameters: explicit env var override > PHYSICS_PROFILE default
GAZEBO_PHYSICS_MAX_STEP_SIZE="${GAZEBO_PHYSICS_MAX_STEP_SIZE:-$_PHYSICS_PROFILE_DEFAULT_STEP}"
GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE="${GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE:-$_PHYSICS_PROFILE_DEFAULT_RATE}"
GAZEBO_PHYSICS_ODE_ITERS="${GAZEBO_PHYSICS_ODE_ITERS:-$_PHYSICS_PROFILE_DEFAULT_ITERS}"
GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL="${GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL:-5.0}"
# Compute theoretical RTF target (max_step_size * real_time_update_rate)
_GAZEBO_PHYSICS_RTF_PRODUCT="$(echo "${GAZEBO_PHYSICS_MAX_STEP_SIZE} * ${GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE}" | bc -l 2>/dev/null || echo '1.0')"

# Competition mode: preserve legacy defaults (0.002 / 500 / ODE 40) when the
# user has not explicitly set PHYSICS_PROFILE or any GAZEBO_PHYSICS_* variable.
# This avoids silently applying earth-normal parameters to an unvalidated mode.
# Explicit overrides (PHYSICS_PROFILE=fidelity, GAZEBO_PHYSICS_ODE_ITERS=30,
# etc.) are always honoured and logged.
if [ "$WORLD_MODE" = "competition" ] && [ "$_PHYSICS_USER_CONFIGURED" = "0" ]; then
  GAZEBO_PHYSICS_MAX_STEP_SIZE="0.002"
  GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE="500"
  GAZEBO_PHYSICS_ODE_ITERS="40"
  _GAZEBO_PHYSICS_RTF_PRODUCT="$(echo "${GAZEBO_PHYSICS_MAX_STEP_SIZE} * ${GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE}" | bc -l 2>/dev/null || echo '1.0')"
  _PHYSICS_EFFECTIVE_SOURCE="competition_legacy_default"
else
  _PHYSICS_EFFECTIVE_SOURCE="${PHYSICS_PROFILE}"
fi

ROBOT_X="${ROBOT_X:-0.0}"
ROBOT_Y="${ROBOT_Y:-2.3}"
ROBOT_Z="${ROBOT_Z:-0.6}"
ROBOT_YAW="${ROBOT_YAW:-1.5708}"
CONTROLLER_BIN="$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl"

# Fail before cleanup, scene generation, or Gazebo startup when the controller
# build is missing.  Without this check, the later background command exits
# with 127 and `wait` makes the terminal appear to close unexpectedly.
if [ "$START_CONTROLLER" = "1" ] && [ ! -x "$CONTROLLER_BIN" ]; then
  echo "ERROR: junior_ctrl is not built: $CONTROLLER_BIN" >&2
  echo "  Rebuild the Unitree controller, then source devel/setup.bash before running auto.sh." >&2
  echo "  The simulation was not started and the current generated scene was left unchanged." >&2
  exit 1
fi

CURRENT_STAGE="STAGE_0_ENV_VALIDATION"
STAGE_STARTED_AT=0
LAST_OBSERVATION="not checked"
CLEANUP_DONE=0
EXIT_STATUS=0
LAUNCH_PID=""
BUILDING_CONTROL_PID=""
CONTROLLER_PID=""
ADAPTER_PID=""
FAST_LIO2_PID=""
SUPERVISOR_PID=""
NAVIGATION_PID=""
RECORDER_PID=""

wall_now() {
  date +%s
}

stage_enter() {
  CURRENT_STAGE="$1"
  STAGE_STARTED_AT="$(wall_now)"
  echo "[STARTUP][$CURRENT_STAGE][ENTER]"
}

stage_pass() {
  echo "[STARTUP][$CURRENT_STAGE][PASS] elapsed_wall=$(( $(wall_now) - STAGE_STARTED_AT ))s${1:+ detail=$1}"
}

startup_fail() {
  local reason="$1"
  echo "[STARTUP][$CURRENT_STAGE][FAIL] reason=$reason last_observation=$(shell_quote "$LAST_OBSERVATION")" >&2
  EXIT_STATUS=1
  exit 1
}

wait_until() {
  local timeout_sec="$1"
  local condition="$2"
  shift 2
  local started now
  started="$(wall_now)"
  echo "[STARTUP][$CURRENT_STAGE][WAIT] condition=$condition timeout=${timeout_sec}s"
  while true; do
    if "$@"; then
      return 0
    fi
    now="$(wall_now)"
    if [ $((now - started)) -ge "$timeout_sec" ]; then
      return 1
    fi
    sleep "$STARTUP_POLL_INTERVAL"
  done
}

ros_master_ready() {
  LAST_OBSERVATION="ROS master unavailable"
  if timeout 2 rosparam list >/dev/null 2>&1; then
    LAST_OBSERVATION="ROS master reachable"
    return 0
  fi
  return 1
}

service_ready() {
  local service="$1"
  LAST_OBSERVATION="service $service unavailable"
  if timeout 2 rosservice info "$service" >/dev/null 2>&1; then
    LAST_OBSERVATION="service $service available"
    return 0
  fi
  return 1
}

node_ready() {
  local node="$1"
  LAST_OBSERVATION="node $node unavailable"
  if timeout 2 rosnode ping -c 1 "$node" >/dev/null 2>&1; then
    LAST_OBSERVATION="node $node responsive"
    return 0
  fi
  return 1
}

topic_has_endpoint() {
  local topic="$1" section="$2" info
  info="$(timeout 2 rostopic info "$topic" 2>/dev/null || true)"
  LAST_OBSERVATION="$topic has no ${section,,}"
  if printf '%s\n' "$info" | awk -v section="$section" '
      $0 ~ ("^" section ":") { active=1; next }
      active && /^[A-Za-z]+:/ { active=0 }
      active && /^[[:space:]]*\*/ { found=1 }
      END { exit(found ? 0 : 1) }'; then
    LAST_OBSERVATION="$topic has ${section,,}"
    return 0
  fi
  return 1
}

topic_has_publisher() {
  topic_has_endpoint "$1" Publishers
}

topic_has_subscriber() {
  topic_has_endpoint "$1" Subscribers
}

topic_message() {
  timeout --kill-after=1s "$TOPIC_MESSAGE_TIMEOUT" rostopic echo -n 1 "$1" 2>/dev/null
}

topic_fresh() {
  local topic="$1" sample first second
  sample="$(timeout --kill-after=1s "$TOPIC_MESSAGE_TIMEOUT" rostopic echo -n 2 "$topic" 2>/dev/null || true)"
  first="$(printf '%s\n' "$sample" | awk '/^[[:space:]]*secs:/{s=$2} /^[[:space:]]*nsecs:/{print s "." $2}' | sed -n '1p')"
  second="$(printf '%s\n' "$sample" | awk '/^[[:space:]]*secs:/{s=$2} /^[[:space:]]*nsecs:/{print s "." $2}' | sed -n '2p')"
  LAST_OBSERVATION="$topic stamps first=${first:-missing} second=${second:-missing}"
  [ -n "$first" ] && [ -n "$second" ] && [ "$first" != "$second" ]
}

pointcloud_nonempty() {
  local topic="$1" sample topic_type count
  topic_type="$(timeout 2 rostopic type "$topic" 2>/dev/null || true)"
  LAST_OBSERVATION="$topic empty or unavailable"
  case "$topic_type" in
    sensor_msgs/PointCloud2)
      count="$(topic_message "$topic" | awk '/^width:/{print $2; exit}' || true)"
      ;;
    sensor_msgs/PointCloud)
      sample="$(topic_message "$topic/points" || true)"
      count="$(printf '%s\n' "$sample" | grep -c '^- ' || true)"
      ;;
    *CustomMsg)
      count="$(topic_message "$topic" | awk '/^point_num:/{print $2; exit}' || true)"
      ;;
    *)
      sample="$(topic_message "$topic" || true)"
      count="$(printf '%s\n' "$sample" | grep -Ec '^points:|^data: \[[^]]|^width: [1-9]' || true)"
      ;;
  esac
  if [ -n "$count" ] && [ "$count" -gt 0 ] 2>/dev/null; then
    LAST_OBSERVATION="$topic has non-empty message"
    return 0
  fi
  return 1
}

imu_finite_upright() {
  local sample z
  sample="$(topic_message /trunk_imu || true)"
  z="$(printf '%s\n' "$sample" | awk '/linear_acceleration:/{in_acc=1; next} in_acc && /^[[:space:]]+z:/{print $2; exit}')"
  LAST_OBSERVATION="/trunk_imu linear_acceleration.z=${z:-missing}"
  [ -n "$z" ] && awk -v value="$z" 'BEGIN { exit(value == value && value > 7.0 && value < 12.5 ? 0 : 1) }'
}

odom_finite() {
  local topic="$1" sample
  sample="$(topic_message "$topic" || true)"
  LAST_OBSERVATION="$topic missing finite pose"
  if printf '%s\n' "$sample" | grep -q 'position:' && \
     ! printf '%s\n' "$sample" | grep -Eqi '(^|[^a-z])(nan|inf)([^a-z]|$)'; then
    LAST_OBSERVATION="$topic finite pose observed"
    return 0
  fi
  return 1
}

topic_value_is() {
  local topic="$1" expected="$2" value
  value="$(topic_message "$topic" | awk '/^data:/{print $2; exit}' || true)"
  value="${value,,}"
  expected="${expected,,}"
  LAST_OBSERVATION="$topic data=${value:-missing}, expected=$expected"
  [ "$value" = "$expected" ]
}

clock_value_ns() {
  topic_message /clock | awk '/^[[:space:]]*secs:/{s=$2} /^[[:space:]]*nsecs:/{printf "%.0f\n", s * 1000000000 + $2; exit}'
}

clock_advancing() {
  local first second
  first="$(clock_value_ns || true)"
  sleep 0.5
  second="$(clock_value_ns || true)"
  LAST_OBSERVATION="/clock first=${first:-missing} second=${second:-missing}"
  [ -n "$first" ] && [ -n "$second" ] && [ "$second" -gt "$first" ]
}

robot_model_ready() {
  local response
  response="$(timeout 3 rosservice call /gazebo/get_model_properties a1_gazebo 2>/dev/null || true)"
  LAST_OBSERVATION="Gazebo model a1_gazebo unavailable"
  if printf '%s\n' "$response" | grep -q 'success: True'; then
    LAST_OBSERVATION="Gazebo model a1_gazebo present"
    return 0
  fi
  return 1
}

controllers_ready() {
  local response count
  response="$(timeout 4 rosservice call /a1_gazebo/controller_manager/list_controllers 2>/dev/null || true)"
  count="$(printf '%s\n' "$response" | grep -c 'state:.*running' || true)"
  LAST_OBSERVATION="running joint controllers=$count/13"
  [ "$count" -ge 13 ]
}

tf_ready() {
  local target="$1" source="$2" sample
  LAST_OBSERVATION="TF $target <- $source unavailable"
  sample="$(timeout 4 rosrun tf tf_echo "$target" "$source" 2>/dev/null || true)"
  if printf '%s\n' "$sample" | grep -q 'Translation:'; then
    LAST_OBSERVATION="TF $target <- $source resolved"
    return 0
  fi
  return 1
}

request_state() {
  local request_topic="$1" type="$2" value="$3" output_topic="$4" timeout_sec="$5"
  local started publisher_pid result=1
  started="$(wall_now)"
  if ! wait_until "$timeout_sec" "subscriber:$request_topic" topic_has_subscriber "$request_topic"; then
    return 1
  fi
  rostopic pub -r 10 "$request_topic" "$type" "data: $value" >/dev/null 2>&1 &
  publisher_pid=$!
  while [ $(( $(wall_now) - started )) -lt "$timeout_sec" ]; do
    if topic_value_is "$output_topic" "$value"; then
      result=0
      break
    fi
  done
  kill -INT "$publisher_pid" 2>/dev/null || true
  wait "$publisher_pid" 2>/dev/null || true
  return "$result"
}

state_stable() {
  local topic="$1" expected="$2" samples="${3:-3}" count=0
  while [ "$count" -lt "$samples" ]; do
    topic_value_is "$topic" "$expected" || return 1
    count=$((count + 1))
    sleep 0.35
  done
}

tracked_process_alive() {
  local pid="$1" label="$2"
  LAST_OBSERVATION="$label pid=${pid:-missing} not alive"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    LAST_OBSERVATION="$label pid=$pid alive"
    return 0
  fi
  return 1
}

topic_receives_messages() {
  local topic="$1" count="${2:-2}" sample observed
  sample="$(timeout --kill-after=1s 6 rostopic echo -n "$count" "$topic" 2>/dev/null || true)"
  observed="$(printf '%s\n' "$sample" | grep -c '^---$' || true)"
  LAST_OBSERVATION="$topic received_messages=$observed/$count"
  [ "$observed" -ge "$count" ]
}

controller_runtime_alive() {
  if [ -n "$CONTROLLER_PID" ]; then
    tracked_process_alive "$CONTROLLER_PID" junior_ctrl
    return
  fi
  if [ "$TERMINAL_BACKEND" = "tmux" ] && command -v tmux >/dev/null 2>&1 && \
     tmux has-session -t "${TMUX_SESSION_PREFIX}-junior_ctrl" 2>/dev/null; then
    LAST_OBSERVATION="junior_ctrl tmux session alive"
    return 0
  fi
  LAST_OBSERVATION="junior_ctrl terminal wrapper/session unavailable"
  return 1
}

# ---------------------------------------------------------------------------
# Launch a command in a dedicated terminal window.  This gives the command a
# real TTY so interactive keyboard input works (e.g. junior_ctrl). tmux is
# the default backend: it preserves the command and its TTY even if a GUI
# terminal launched from Snap Code flashes and closes.
# Start gnome-terminal.real directly in a clean desktop environment: when
# auto.sh is launched from Snap Code, the gnome-terminal D-Bus wrapper inherits
# Snap libraries and fails before a window is created.  Do not alter the
# system terminal or ROS environment; only sanitise the terminal launcher.
# Falls back to background execution when no graphical terminal is available.
# ---------------------------------------------------------------------------
launch_in_terminal() {
  local title="$1"
  local command="$2"
  local env_block tmux_session tmux_command tmux_script
  local -a terminal_env
  env_block="export UNITREE_CTRL_DT='${UNITREE_CTRL_DT}';"
  env_block="${env_block} export GAZEBO_MODEL_PATH='${GAZEBO_MODEL_PATH:-}:${SCENE_OUTPUT_DIR}:${UNITREE_GAZEBO_MODELS}';"
  env_block="${env_block} export GAZEBO_PLUGIN_PATH='${GAZEBO_PLUGIN_PATH:-}';"
  env_block="${env_block} export ROS_PACKAGE_PATH='${ROS_PACKAGE_PATH:-}';"
  env_block="${env_block} export CMAKE_PREFIX_PATH='${CMAKE_PREFIX_PATH:-}';"
  env_block="${env_block} export PYTHONPATH='${PYTHONPATH:-}';"
  if [ -n "${RL_POLICY_PATH:-}" ]; then
    env_block="${env_block} export RL_POLICY_PATH=$(shell_quote "$RL_POLICY_PATH");"
  fi
  if [ -n "${UNITREE_RL_DIAG_PATH:-}" ]; then
    env_block="${env_block} export UNITREE_RL_DIAG_PATH=$(shell_quote "$UNITREE_RL_DIAG_PATH");"
  fi
  for thread_env_name in OMP_NUM_THREADS MKL_NUM_THREADS OPENBLAS_NUM_THREADS NUMEXPR_NUM_THREADS; do
    if [ -n "${!thread_env_name:-}" ]; then
      env_block="${env_block} export ${thread_env_name}=$(shell_quote "${!thread_env_name}");"
    fi
  done

  terminal_env=(
    env -i
    "HOME=${HOME:-$WORKSPACE_DIR}"
    "USER=${USER:-$(id -un)}"
    "LOGNAME=${LOGNAME:-${USER:-$(id -un)}}"
    "DISPLAY=${DISPLAY:-}"
    "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-}"
    "XAUTHORITY=${XAUTHORITY:-}"
    "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-}"
    "DBUS_SESSION_BUS_ADDRESS=${DBUS_SESSION_BUS_ADDRESS:-}"
    "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
  )

  if [ "$TERMINAL_BACKEND" = "tmux" ] && command -v tmux >/dev/null 2>&1; then
    tmux_session="${TMUX_SESSION_PREFIX}-${title}"
    tmux kill-session -t "$tmux_session" 2>/dev/null || true
    tmux_command="
      source /opt/ros/noetic/setup.bash
      source '${WORKSPACE_DIR}/devel/setup.bash'
      cd '${WORKSPACE_DIR}'
      ${env_block}
      echo '=== ${title} (tmux session: ${tmux_session}) ==='
      echo 'Command: ${command}'
      echo ''
      ( ${command} )
      echo ''
      echo '=== ${title} exited. This tmux session is kept open for diagnostics. Type exit to close. ==='
      exec bash --noprofile --norc -i
    "
    # tmux executes its command through /bin/sh. Do not pass this multiline
    # Bash script through printf %q: Bash's $'...' escaping is not portable to
    # /bin/sh and would make the session exit immediately. Keep a runtime
    # script under logs instead; it also preserves diagnostics for reattach.
    tmux_script="${WORKSPACE_DIR}/logs/${title}.tmux.sh"
    printf '%s\n%s\n' '#!/usr/bin/env bash' "$tmux_command" > "$tmux_script"
    chmod 700 "$tmux_script"
    if tmux new-session -d -s "$tmux_session" "bash '$tmux_script'"; then
      printf '%s\n' "$tmux_session" > "${WORKSPACE_DIR}/logs/${title}.tmux_session"
      echo "tmux session ready: $tmux_session"
      echo "  Reattach from any terminal: tmux attach-session -t $tmux_session"
      if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && [ -x /usr/bin/gnome-terminal.real ]; then
        # Use tmux as the terminal's top-level process. When a later auto.sh
        # startup removes this session, the attach client and its window exit
        # instead of falling through to a leftover interactive Bash shell.
        "${terminal_env[@]}" /usr/bin/gnome-terminal.real --title="$title" -- \
          tmux attach-session -t "$tmux_session" &
        echo $! > "${WORKSPACE_DIR}/logs/${title}.pid"
      fi
      return
    fi
    echo "WARNING: Failed to create tmux session '$tmux_session'; falling back to direct terminal launch." >&2
  fi

  if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && [ -x /usr/bin/gnome-terminal.real ]; then
    "${terminal_env[@]}" /usr/bin/gnome-terminal.real --title="$title" -- bash -c "
      source /opt/ros/noetic/setup.bash
      source '${WORKSPACE_DIR}/devel/setup.bash'
      cd '${WORKSPACE_DIR}'
      ${env_block}
      echo '=== ${title} ==='
      echo 'Command: ${command}'
      echo ''
      # rosrun ends with exec; isolate it so it cannot replace this terminal
      # shell and bypass the diagnostic shell below when RViz exits or fails.
      ( ${command} )
      echo ''
      echo '=== ${title} exited. This terminal is kept open for diagnostics. Type exit to close. ==='
      exec bash --noprofile --norc -i
    " &
  elif command -v xterm >/dev/null 2>&1; then
    xterm -title "$title" -e "bash -c '
      source /opt/ros/noetic/setup.bash
      source '${WORKSPACE_DIR}/devel/setup.bash'
      cd '${WORKSPACE_DIR}'
      ${env_block}
      echo === ${title} ===
      # Keep rosrun's exec confined to a child shell.
      ( ${command} )
      echo ''
      echo === ${title} exited. This terminal is kept open for diagnostics. Type exit to close. ===
      exec bash --noprofile --norc -i
    '" &
  else
    echo "WARNING: No graphical terminal found; launching '$title' in background." >&2
    bash -c "
      source /opt/ros/noetic/setup.bash
      source '${WORKSPACE_DIR}/devel/setup.bash'
      cd '${WORKSPACE_DIR}'
      ${env_block}
      ${command}
    " > "${WORKSPACE_DIR}/logs/${title}.log" 2>&1 &
  fi
  # Record the PID of the terminal wrapper / background process.
  echo $! > "${WORKSPACE_DIR}/logs/${title}.pid"
}

cleanup_tmux_sessions() {
  local title session
  for title in junior_ctrl rviz; do
    session="${TMUX_SESSION_PREFIX}-${title}"
    if command -v tmux >/dev/null 2>&1 && tmux has-session -t "$session" 2>/dev/null; then
      echo "Stopping previous tmux session: $session"
      tmux kill-session -t "$session" 2>/dev/null || true
    fi
    # These are runtime records, not user configuration. Remove stale records
    # after the associated session is gone so a new startup cannot mistake
    # them for its freshly launched terminal.
    rm -f "${WORKSPACE_DIR}/logs/${title}.tmux_session" "${WORKSPACE_DIR}/logs/${title}.pid"
  done
}

stop_tracked_pid() {
  local pid="${1:-}" label="${2:-process}" attempt signal tree_pid
  local -a tree_pids
  [ -n "$pid" ] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  echo "[CLEANUP] stopping $label pid=$pid"
  for signal in INT TERM KILL; do
    # Snapshot the complete tree before roslaunch exits and reparents its
    # grandchildren; direct pkill -P only covers one level.
    mapfile -t tree_pids < <(
      local -a queue=("$pid")
      local current child
      while [ "${#queue[@]}" -gt 0 ]; do
        current="${queue[0]}"
        queue=("${queue[@]:1}")
        printf '%s\n' "$current"
        while read -r child; do
          [ -n "$child" ] && queue+=("$child")
        done < <(ps -o pid= --ppid "$current" 2>/dev/null | tr -d ' ')
      done
    )
    for tree_pid in "${tree_pids[@]}"; do
      kill -"$signal" "$tree_pid" 2>/dev/null || true
    done
    for attempt in $(seq 1 20); do
      kill -0 "$pid" 2>/dev/null || return 0
      sleep 0.1
    done
  done
}

pid_from_owned_file() {
  local file="$1" pid cmdline
  [ -f "$file" ] || return 1
  pid="$(tr -dc '0-9' < "$file")"
  [ -n "$pid" ] && [ -r "/proc/$pid/cmdline" ] || return 1
  cmdline="$(tr '\0' ' ' < "/proc/$pid/cmdline")"
  if [[ "$cmdline" == *"$WORKSPACE_DIR"* ]]; then
    printf '%s' "$pid"
    return 0
  fi
  echo "[CLEANUP] refusing unowned stale pid=$pid file=$file cmd=$(shell_quote "$cmdline")" >&2
  return 1
}

cleanup_previous_owned_run() {
  local file pid
  echo "Checking for processes owned by a previous run in this worktree..."
  for file in \
    "$WORKSPACE_DIR/logs/navigation.pid" \
    "$WORKSPACE_DIR/logs/nav_state_supervisor.pid" \
    "$WORKSPACE_DIR/logs/fast_lio2.pid" \
    "$WORKSPACE_DIR/logs/scan_adapter.pid" \
    "$WORKSPACE_DIR/logs/building_control.pid" \
    "$WORKSPACE_DIR/logs/competition_gazebo.pid"; do
    pid="$(pid_from_owned_file "$file" || true)"
    [ -z "$pid" ] || stop_tracked_pid "$pid" "stale $(basename "$file")"
  done
  cleanup_tmux_sessions
}

if [ "$SKIP_GLOBAL_PROCESS_CLEANUP" = "true" ]; then
  echo "Skipping previous-run cleanup (SKIP_GLOBAL_PROCESS_CLEANUP=true)."
else
  cleanup_previous_owned_run
fi

echo "Checking ROS Python environment for Conda contamination..."
ROS_PYTHON_INFO="$(
    /usr/bin/python3 -c \
    'import sys, xml.dom.minidom; print(sys.executable); print(xml.dom.minidom.__file__)'
)"

echo "ROS Python environment:"
echo "${ROS_PYTHON_INFO}"

if echo "${ROS_PYTHON_INFO}" | grep -qiE 'miniconda|anaconda'; then
    echo "ERROR: Conda Python is still contaminating the ROS environment." >&2
    exit 1
fi

echo "Sourcing ROS environment..."
source /opt/ros/noetic/setup.bash
if [ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
  echo "Missing $WORKSPACE_DIR/devel/setup.bash. Run catkin_make in this workspace before starting the simulation." >&2
  exit 1
fi
source "$WORKSPACE_DIR/devel/setup.bash"

# ── Verify external dependencies ──
# FAST_LIO and livox_ros_driver are external ROS packages not committed to
# this repository.  They must be prepared before the first build via:
#
#   tools/prepare_shared_ros_deps.sh --prepare
#
# The script validates shared source checkouts, wires symlinks under
# src/external/, and prepares the Livox-SDK prefix.
_MISSING_EXTERNAL=0
if [ "$ENABLE_FAST_LIO2" = "true" ] || [ "$ENABLE_NAVIGATION" = "true" ]; then
  if ! rospack find fast_lio >/dev/null 2>&1; then
    echo "ERROR: FAST_LIO package not found.  Run: tools/prepare_shared_ros_deps.sh --prepare" >&2
    _MISSING_EXTERNAL=1
  fi
  if ! rospack find livox_ros_driver >/dev/null 2>&1; then
    echo "ERROR: livox_ros_driver package not found.  Run: tools/prepare_shared_ros_deps.sh --prepare" >&2
    _MISSING_EXTERNAL=1
  fi
  if [ "$_MISSING_EXTERNAL" -ne 0 ]; then
    exit 1
  fi
fi

export ROS_PACKAGE_PATH="$WORKSPACE_DIR/src:${ROS_PACKAGE_PATH:-}"
export CMAKE_PREFIX_PATH="$WORKSPACE_DIR/devel:${CMAKE_PREFIX_PATH:-}"
export PYTHONPATH="$WORKSPACE_DIR/src/building_generator_classic:$WORKSPACE_DIR/src/building_generator_core:${PYTHONPATH:-}"

echo "Runtime source diagnostics:"
echo "  Workspace: $WORKSPACE_DIR"
if command -v git >/dev/null 2>&1 && git -C "$WORKSPACE_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "  Git branch: $(git -C "$WORKSPACE_DIR" branch --show-current 2>/dev/null || true)"
  echo "  Git HEAD:   $(git -C "$WORKSPACE_DIR" rev-parse --short HEAD 2>/dev/null || true)"
  if git -C "$WORKSPACE_DIR" merge-base --is-ancestor 69ff34e7 HEAD 2>/dev/null; then
    echo "  Contains FAST-LIO2 pointcloud semantics fix 69ff34e7: yes"
  else
    echo "  WARNING: HEAD does not contain FAST-LIO2 pointcloud semantics fix 69ff34e7" >&2
  fi
fi
echo "  ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
echo "  CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"
if command -v rospack >/dev/null 2>&1; then
  echo "  simenv_fast_lio2_integration: $(rospack find simenv_fast_lio2_integration 2>/dev/null || echo NOT_FOUND)"
  echo "  a1_description: $(rospack find a1_description 2>/dev/null || echo NOT_FOUND)"
  echo "  fast_lio: $(rospack find fast_lio 2>/dev/null || echo NOT_FOUND)"
fi
echo "  scan_to_pointcloud2.py: $WORKSPACE_DIR/src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py"

GENERATOR_SCRIPT="$WORKSPACE_DIR/src/building_obstacles/scripts/generate_competition_scene.py"
BUILDING_CONTROL_SCRIPT="$WORKSPACE_DIR/src/building_generator_classic/scripts/building_generator_classic_control"
UNITREE_GAZEBO_MODELS="$WORKSPACE_DIR/src/unitree_guide/unitree_ros/unitree_gazebo/models"
SCENE_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"
RESULTS_DIR="$WORKSPACE_DIR/results"
EARTH_WORLD_FILE="$WORKSPACE_DIR/src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world"
mkdir -p "$WORKSPACE_DIR/logs"

if [ "$WORLD_MODE" = "competition" ]; then
  mkdir -p "$SCENE_OUTPUT_DIR" "$RESULTS_DIR"
  echo "Generating competition scene..."
  GENERATOR_ARGS=(
    --output-dir "$SCENE_OUTPUT_DIR"
    --results-dir "$RESULTS_DIR"
    --floor-count "$FLOOR_COUNT"
    --rooms-per-floor "$ROOMS_PER_FLOOR"
    --width "$BUILDING_WIDTH"
    --length "$BUILDING_LENGTH"
    --danger-count "$DANGER_COUNT"
    --distractor-count "$DISTRACTOR_COUNT"
    --robot-x "$ROBOT_X"
    --robot-y "$ROBOT_Y"
    --robot-z "$ROBOT_Z"
    --robot-yaw "$ROBOT_YAW"
  )
  if [ -n "$SEED" ]; then
    GENERATOR_ARGS+=(--seed "$SEED")
  fi
  GENERATOR_ARGS+=(--physics-max-step-size "$GAZEBO_PHYSICS_MAX_STEP_SIZE")
  GENERATOR_ARGS+=(--physics-real-time-update-rate "$GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE")
  GENERATOR_ARGS+=(--physics-ode-iters "$GAZEBO_PHYSICS_ODE_ITERS")
  GENERATOR_ARGS+=(--physics-contact-max-correcting-vel "$GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL")
  if [ "$WRITE_GENERATED_TRUTH_COPY" = "false" ]; then
    GENERATOR_ARGS+=(--no-generated-truth-copy)
  fi
  python3 "$GENERATOR_SCRIPT" "${GENERATOR_ARGS[@]}" \
    > "$SCENE_OUTPUT_DIR/scene_manifest.stdout.json"
  export BUILDING_WORLD_FILE="$SCENE_OUTPUT_DIR/competition_scene.world"
else
  # Earth mode: for 'normal' profile, generate a temp world with substituted
  # physics parameters.  For 'fidelity', use earth.world directly since it
  # already contains the required 0.0002 / 5000 / ODE 50 configuration.
  if [ "$PHYSICS_PROFILE" = "normal" ]; then
    mkdir -p "$SCENE_OUTPUT_DIR"
    _EARTH_PHYSICS_WORLD="$SCENE_OUTPUT_DIR/earth_physics.world"
    cp "$EARTH_WORLD_FILE" "$_EARTH_PHYSICS_WORLD"
    sed -i "s|<max_step_size>[0-9.]*</max_step_size>|<max_step_size>${GAZEBO_PHYSICS_MAX_STEP_SIZE}</max_step_size>|" "$_EARTH_PHYSICS_WORLD"
    sed -i "s|<real_time_update_rate>[0-9]*</real_time_update_rate>|<real_time_update_rate>${GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE}</real_time_update_rate>|" "$_EARTH_PHYSICS_WORLD"
    sed -i "s|<iters>[0-9]*</iters>|<iters>${GAZEBO_PHYSICS_ODE_ITERS}</iters>|" "$_EARTH_PHYSICS_WORLD"
    export BUILDING_WORLD_FILE="$_EARTH_PHYSICS_WORLD"
    echo "Generated earth physics world: $_EARTH_PHYSICS_WORLD"
  else
    export BUILDING_WORLD_FILE="$EARTH_WORLD_FILE"
  fi
  if [ ! -f "$BUILDING_WORLD_FILE" ]; then
    echo "ERROR: Earth world file not found: $BUILDING_WORLD_FILE" >&2
    exit 1
  fi
  echo "Skipping competition scene generation for WORLD_MODE=earth."
fi
export COMPETITION_ROBOT_X="$ROBOT_X"
export COMPETITION_ROBOT_Y="$ROBOT_Y"
export COMPETITION_ROBOT_Z="$ROBOT_Z"
export COMPETITION_ROBOT_YAW="$ROBOT_YAW"
export UNITREE_CTRL_DT
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$SCENE_OUTPUT_DIR:$UNITREE_GAZEBO_MODELS"
export GAZEBO_PLUGIN_PATH="$WORKSPACE_DIR/devel/lib:${GAZEBO_PLUGIN_PATH:-}"

echo "=========================================="
echo "Startup summary"
echo "  World mode: $WORLD_MODE"
echo "  Physics profile: ${_PHYSICS_EFFECTIVE_SOURCE:-$PHYSICS_PROFILE}"
if [ "$_PHYSICS_USER_CONFIGURED" = "1" ]; then
  echo "  Physics source:  explicit user configuration"
elif [ "$WORLD_MODE" = "competition" ] && [ "${_PHYSICS_EFFECTIVE_SOURCE:-}" = "competition_legacy_default" ]; then
  echo "  Physics source:  competition legacy default (0.002/500/40)"
fi
echo "  Workspace: $WORKSPACE_DIR"
echo "  World:   $BUILDING_WORLD_FILE"
echo "  Robot spawn pose: x=$ROBOT_X y=$ROBOT_Y z=$ROBOT_Z yaw=$ROBOT_YAW"
echo "  GUI: $GUI"
if [ "$WORLD_MODE" = "competition" ]; then
  echo "  Truth:   $RESULTS_DIR/danger_truth.json"
  echo "  Manifest:$SCENE_OUTPUT_DIR/scene_manifest.json"
  echo "  Result:  $RESULTS_DIR/detected_danger.json"
else
  echo "  Truth:   disabled"
  echo "  Manifest:disabled"
  echo "  Result:  disabled"
fi
echo "  Sensor data: $ENABLE_SENSOR_DATA"
if [ "$ENABLE_POINTCLOUD_CONVERTER" = "true" ]; then
  echo "  Legacy PointCloud/Livox converter: enabled"
else
  echo "  Legacy PointCloud/Livox converter: disabled"
fi
echo "  LiDAR visualization: $ENABLE_LIDAR_VISUALIZATION"
echo "  Ground truth topics: $ENABLE_GROUND_TRUTH"
echo "  Referee odom: $ENABLE_REFEREE_ODOM"
echo "  FAST-LIO2 mapping: $ENABLE_FAST_LIO2"
echo "  FAST-LIO2 adapter: $ENABLE_FAST_LIO2"
echo "  FAST-LIO2 lidar input: /scan_pointcloud2"
echo "  RViz: $ENABLE_RVIZ"
if [ "$ENABLE_NAVIGATION" = "true" ]; then
echo "  ---- Navigation ----"
echo "  Navigation enabled: $ENABLE_NAVIGATION"
echo "  Navigation mode: $NAV_MODE"
if [ "$NAV_MODE" = "dsv_falco" ]; then
echo "  DSV enabled: true"
else
echo "  DSV enabled: false"
fi
echo "  Max linear x: $NAV_MAX_LINEAR_X"
echo "  Max angular z: $NAV_MAX_ANGULAR_Z"
echo "  Command timeout: $NAV_COMMAND_TIMEOUT"
echo "  Auto trotting: $NAV_AUTO_TROTTING"
echo "  Auto navigation enable: $NAV_AUTO_ENABLE"
echo "  Auto exploration start: $NAV_AUTO_START_EXPLORATION"
echo "  Navigation log: $WORKSPACE_DIR/logs/navigation.log"
fi
	if [ "$ENABLE_EXPLORATION_RECORDING" = "true" ]; then
	echo "  ---- Exploration Recording ----"
	echo "  Recording enabled: $ENABLE_EXPLORATION_RECORDING"
	echo "  Run ID: $EXPLORATION_RUN_ID"
	echo "  Output dir: $EXPLORATION_OUTPUT_DIR"
	echo "  Max sim time: $EXPLORATION_MAX_SIM_TIME s"
	echo "  Finish quiet time: $EXPLORATION_FINISH_QUIET_TIME s"
	fi
echo "  Building controller: $START_BUILDING_CONTROL"
echo "  Virtual joystick: $START_VIRTUAL_JOY"
echo "  Timing diagnostics: $TIMING_DIAGNOSTICS_ENABLED"
if [ "$TIMING_DIAGNOSTICS_ENABLED" = "true" ]; then
  echo "  Timing diagnostics path: $TIMING_DIAGNOSTICS_PATH"
fi
if [ -n "${RL_POLICY_PATH:-}" ]; then
  echo "  RL policy override: $RL_POLICY_PATH"
else
  echo "  RL policy override: unset (controller default; /rl_policy_path ROS param still has priority)"
fi
if [ -n "${UNITREE_RL_DIAG_PATH:-}" ]; then
  echo "  RL deployment diagnostics path: $UNITREE_RL_DIAG_PATH"
fi
for thread_env_name in OMP_NUM_THREADS MKL_NUM_THREADS OPENBLAS_NUM_THREADS NUMEXPR_NUM_THREADS; do
  if [ -n "${!thread_env_name:-}" ]; then
    echo "  ${thread_env_name}: ${!thread_env_name}"
  fi
done
echo "  Gazebo starts paused: $PAUSED"
echo "  Auto unpause: $AUTO_UNPAUSE (as soon as Gazebo unpause service is ready)"
echo "  Gazebo physics:"
echo "    max_step_size:            $GAZEBO_PHYSICS_MAX_STEP_SIZE"
echo "    real_time_update_rate:    $GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE"
echo "    ode_iters:                $GAZEBO_PHYSICS_ODE_ITERS"
echo "    contact_max_correcting_vel: $GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL"
echo "    theoretical target RTF:   $_GAZEBO_PHYSICS_RTF_PRODUCT"
echo "  Gazebo plugin path: $GAZEBO_PLUGIN_PATH"
echo "  Startup wall-time timeouts:"
echo "    ROS master: $ROS_MASTER_TIMEOUT s"
echo "    Gazebo: $GAZEBO_READY_TIMEOUT s  robot: $ROBOT_READY_TIMEOUT s"
echo "    controller: $CONTROLLER_READY_TIMEOUT s  sensor: $SENSOR_READY_TIMEOUT s"
echo "    FAST-LIO2: $FAST_LIO2_READY_TIMEOUT s  supervisor: $NAV_SUPERVISOR_READY_TIMEOUT s"
echo "    navigation: $NAVIGATION_READY_TIMEOUT s  state transition: $STATE_TRANSITION_TIMEOUT s"
echo "=========================================="

cleanup() {
  [ "$CLEANUP_DONE" = "0" ] || return 0
  CLEANUP_DONE=1
  trap - EXIT INT TERM
  echo ""
  echo "[CLEANUP] begin status=$EXIT_STATUS stage=$CURRENT_STAGE"
  if command -v rostopic >/dev/null 2>&1 && ros_master_ready; then
    timeout 1 rostopic pub -r 10 /navigation/request_exploring std_msgs/Bool "data: false" >/dev/null 2>&1 || true
    timeout 1 rostopic pub -r 10 /navigation/request_enabled std_msgs/Bool "data: false" >/dev/null 2>&1 || true
    timeout 1 rostopic pub -r 10 /navigation/request_fsm_state std_msgs/Int8 "data: 2" >/dev/null 2>&1 || true
    timeout 1 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
  fi
  stop_tracked_pid "$RECORDER_PID" recorder
  stop_tracked_pid "$NAVIGATION_PID" navigation
  stop_tracked_pid "$FAST_LIO2_PID" fast_lio2
  stop_tracked_pid "$ADAPTER_PID" scan_adapter
  stop_tracked_pid "$SUPERVISOR_PID" nav_state_supervisor
  stop_tracked_pid "$CONTROLLER_PID" controller_terminal
  stop_tracked_pid "$BUILDING_CONTROL_PID" building_control
  cleanup_tmux_sessions
  stop_tracked_pid "$LAUNCH_PID" gazebo_roslaunch
  echo "[CLEANUP] complete"
}

trap 'EXIT_STATUS=$?; cleanup' EXIT
trap 'EXIT_STATUS=130; exit 130' INT TERM

stage_enter STAGE_0_ENV_VALIDATION
stage_pass "workspace=$WORKSPACE_DIR"

if [ "$START_VIRTUAL_JOY" = "1" ]; then
  rosrun unitree_guide virtual_joy.py > "$WORKSPACE_DIR/logs/virtual_joy.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/virtual_joy.pid"
fi

stage_enter STAGE_1_ROS_MASTER_READY
echo "Launching Gazebo, Unitree A1 model, sensors, and ROS interfaces..."
setsid roslaunch unitree_guide multi_floor_gazeboSim.launch \
  gui:="$GUI" paused:="$PAUSED" user_debug:=False rname:=a1 \
  robot_x:="$ROBOT_X" robot_y:="$ROBOT_Y" robot_z:="$ROBOT_Z" robot_yaw:="$ROBOT_YAW" \
  enable_sensor_data:="$ENABLE_SENSOR_DATA" \
  enable_lidar_visualization:="$ENABLE_LIDAR_VISUALIZATION" \
  enable_referee_odom:="$ENABLE_REFEREE_ODOM" enable_ground_truth:="$ENABLE_GROUND_TRUTH" \
  enable_foot_force_visual:="$ENABLE_FOOT_FORCE_VISUAL" enable_joy_node:="$ENABLE_JOY_NODE" \
  enable_pointcloud_converter:="$ENABLE_POINTCLOUD_CONVERTER" \
  pointcloud_use_ground_truth_odom:="$POINTCLOUD_USE_GROUND_TRUTH_ODOM" \
  > "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$WORKSPACE_DIR/logs/competition_gazebo.pid"
wait_until "$ROS_MASTER_TIMEOUT" "ROS master reachable" ros_master_ready || startup_fail "ROS master timeout=${ROS_MASTER_TIMEOUT}s"
stage_pass

stage_enter STAGE_2_GAZEBO_READY
wait_until "$GAZEBO_READY_TIMEOUT" "service:/gazebo/get_world_properties" service_ready /gazebo/get_world_properties || startup_fail "Gazebo service timeout"
if [ "$PAUSED" = "true" ]; then
  if [ "$AUTO_UNPAUSE" != "true" ]; then
    startup_fail "PAUSED=true requires AUTO_UNPAUSE=true for state-driven startup"
  fi
  rosservice call /gazebo/unpause_physics >/dev/null 2>&1 || startup_fail "unpause service call failed"
fi
wait_until "$GAZEBO_READY_TIMEOUT" "/clock advancing" clock_advancing || startup_fail "/clock did not advance"
wait_until "$GAZEBO_READY_TIMEOUT" "robot model a1_gazebo" robot_model_ready || startup_fail "robot model missing"
stage_pass

stage_enter STAGE_3_ROBOT_READY
wait_until "$ROBOT_READY_TIMEOUT" "13 joint controllers running" controllers_ready || startup_fail "joint controllers not ready"
wait_until "$ROBOT_READY_TIMEOUT" "publisher:/a1_gazebo/joint_states" topic_has_publisher /a1_gazebo/joint_states || startup_fail "joint-state publisher missing"
wait_until "$ROBOT_READY_TIMEOUT" "live:/a1_gazebo/joint_states" topic_receives_messages /a1_gazebo/joint_states 2 || startup_fail "joint states stale"
wait_until "$ROBOT_READY_TIMEOUT" "publisher:/trunk_imu" topic_has_publisher /trunk_imu || startup_fail "trunk IMU publisher missing"
stage_pass

if [ "$START_BUILDING_CONTROL" = "1" ]; then
  python3 "$BUILDING_CONTROL_SCRIPT" --door-config "$SCENE_OUTPUT_DIR/door_config.yaml" \
    --elevator-config "$SCENE_OUTPUT_DIR/elevator_config.yaml" \
    > "$WORKSPACE_DIR/logs/building_control.log" 2>&1 &
  BUILDING_CONTROL_PID=$!
  echo "$BUILDING_CONTROL_PID" > "$WORKSPACE_DIR/logs/building_control.pid"
fi

if [ "$START_CONTROLLER" = "1" ] || [ "$ENABLE_NAVIGATION" = "true" ]; then
  stage_enter STAGE_4_NAV_SUPERVISOR_READY
  /usr/bin/python3 "$WORKSPACE_DIR/src/navigation/simenv_navigation_bridge/scripts/nav_state_supervisor.py" \
    > "$WORKSPACE_DIR/logs/nav_state_supervisor.log" 2>&1 &
  SUPERVISOR_PID=$!
  echo "$SUPERVISOR_PID" > "$WORKSPACE_DIR/logs/nav_state_supervisor.pid"
  wait_until "$NAV_SUPERVISOR_READY_TIMEOUT" "node:/nav_state_supervisor" node_ready /nav_state_supervisor || startup_fail "supervisor node timeout"
  for request_topic in /navigation/request_enabled /navigation/request_exploring /navigation/request_fsm_state; do
    wait_until "$NAV_SUPERVISOR_READY_TIMEOUT" "subscriber:$request_topic" topic_has_subscriber "$request_topic" || startup_fail "supervisor request subscriber missing: $request_topic"
  done
  for output_topic in /navigation/enabled /navigation/start_exploring /fsm/state_cmd; do
    wait_until "$NAV_SUPERVISOR_READY_TIMEOUT" "publisher:$output_topic" topic_has_publisher "$output_topic" || startup_fail "supervisor output publisher missing: $output_topic"
  done
  request_state /navigation/request_exploring std_msgs/Bool false /navigation/start_exploring "$STATE_TRANSITION_TIMEOUT" || startup_fail "safe exploring=false rejected"
  request_state /navigation/request_enabled std_msgs/Bool false /navigation/enabled "$STATE_TRANSITION_TIMEOUT" || startup_fail "safe navigation=false rejected"
  request_state /navigation/request_fsm_state std_msgs/Int8 2 /fsm/state_cmd "$STATE_TRANSITION_TIMEOUT" || startup_fail "safe FSM=2 rejected"
  stage_pass "enabled=false exploring=false fsm=2"
fi

if [ "$START_CONTROLLER" = "1" ]; then
  stage_enter STAGE_5_CONTROLLER_READY
  if [ "$TIMING_DIAGNOSTICS_ENABLED" = "true" ]; then
    rosparam set /timing_diagnostics_enabled true
    rosparam set /timing_diagnostics_path "$TIMING_DIAGNOSTICS_PATH"
  fi
  launch_in_terminal "junior_ctrl" "$CONTROLLER_BIN"
  CONTROLLER_PID="$(tr -dc '0-9' < "$WORKSPACE_DIR/logs/junior_ctrl.pid" 2>/dev/null || true)"
  wait_until "$CONTROLLER_READY_TIMEOUT" "controller process/session" controller_runtime_alive || startup_fail "controller process exited"
  wait_until "$CONTROLLER_READY_TIMEOUT" "node:/unitree_gazebo_servo" node_ready /unitree_gazebo_servo || startup_fail "junior_ctrl node timeout"
  wait_until "$CONTROLLER_READY_TIMEOUT" "subscriber:/fsm/state_cmd" topic_has_subscriber /fsm/state_cmd || startup_fail "FSM subscriber missing"
  wait_until "$CONTROLLER_READY_TIMEOUT" "live joint feedback" topic_receives_messages /a1_gazebo/FR_hip_controller/state 2 || startup_fail "joint feedback stale"
  stage_pass

  stage_enter STAGE_6_FIXED_STAND_READY
  wait_until "$CONTROLLER_READY_TIMEOUT" "finite upright /trunk_imu" imu_finite_upright || startup_fail "IMU invalid or robot not upright"
  request_state /navigation/request_fsm_state std_msgs/Int8 2 /fsm/state_cmd "$STATE_TRANSITION_TIMEOUT" || startup_fail "FixedStand request rejected"
  state_stable /fsm/state_cmd 2 3 || startup_fail "FSM=2 did not remain stable"
  stage_pass "fsm=2 stable"
fi

if [ "$ENABLE_SENSOR_DATA" = "true" ]; then
  stage_enter STAGE_7_SENSOR_READY
  wait_until "$SENSOR_READY_TIMEOUT" "publisher:/scan" topic_has_publisher /scan || startup_fail "/scan publisher missing"
  wait_until "$SENSOR_READY_TIMEOUT" "non-empty:/scan" pointcloud_nonempty /scan || startup_fail "/scan empty"
  wait_until "$SENSOR_READY_TIMEOUT" "fresh:/scan" topic_fresh /scan || startup_fail "/scan stale"
  wait_until "$SENSOR_READY_TIMEOUT" "finite:/trunk_imu" imu_finite_upright || startup_fail "/trunk_imu invalid"
  wait_until "$SENSOR_READY_TIMEOUT" "fresh:/trunk_imu" topic_fresh /trunk_imu || startup_fail "/trunk_imu stale"
  if [ "$ENABLE_POINTCLOUD_CONVERTER" = "true" ]; then
    wait_until "$SENSOR_READY_TIMEOUT" "publisher:/livox/lidar2" topic_has_publisher /livox/lidar2 || startup_fail "legacy CustomMsg publisher missing"
    wait_until "$SENSOR_READY_TIMEOUT" "publisher:/livox/Pointcloud2" topic_has_publisher /livox/Pointcloud2 || startup_fail "legacy PointCloud2 publisher missing"
  fi
  stage_pass
fi

if [ "$ENABLE_FAST_LIO2" = "true" ]; then
  [ "$ENABLE_SENSOR_DATA" = "true" ] || startup_fail "FAST-LIO2 requires ENABLE_SENSOR_DATA=true"
  stage_enter STAGE_8_FAST_LIO2_ADAPTER_READY
  wait_until "$SENSOR_READY_TIMEOUT" "TF base<-laser_livox" tf_ready base laser_livox || startup_fail "LiDAR TF unavailable"
  setsid /usr/bin/python3 "$WORKSPACE_DIR/src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py" \
    > "$WORKSPACE_DIR/logs/scan_adapter.log" 2>&1 &
  ADAPTER_PID=$!
  echo "$ADAPTER_PID" > "$WORKSPACE_DIR/logs/scan_adapter.pid"
  wait_until "$SENSOR_READY_TIMEOUT" "publisher:/scan_pointcloud2" topic_has_publisher /scan_pointcloud2 || startup_fail "adapter publisher missing"
  wait_until "$SENSOR_READY_TIMEOUT" "non-empty:/scan_pointcloud2" pointcloud_nonempty /scan_pointcloud2 || startup_fail "adapter output empty"
  wait_until "$SENSOR_READY_TIMEOUT" "fresh:/scan_pointcloud2" topic_fresh /scan_pointcloud2 || startup_fail "adapter output stale"
  LAST_OBSERVATION="/scan_pointcloud2 frame_id mismatch"
  topic_message /scan_pointcloud2 | grep -q 'frame_id:.*laser_livox' || startup_fail "adapter frame_id is not laser_livox"
  stage_pass

  stage_enter STAGE_9_FAST_LIO2_READY
  setsid roslaunch simenv_fast_lio2_integration simenv_fast_lio2_mapping.launch enable_adapter:=false \
    > "$WORKSPACE_DIR/logs/fast_lio2.log" 2>&1 &
  FAST_LIO2_PID=$!
  echo "$FAST_LIO2_PID" > "$WORKSPACE_DIR/logs/fast_lio2.pid"
  wait_until "$FAST_LIO2_READY_TIMEOUT" "node:/laserMapping" node_ready /laserMapping || startup_fail "laserMapping node timeout"
  wait_until "$FAST_LIO2_READY_TIMEOUT" "finite:/Odometry" odom_finite /Odometry || startup_fail "/Odometry invalid"
  wait_until "$FAST_LIO2_READY_TIMEOUT" "fresh:/Odometry" topic_fresh /Odometry || startup_fail "/Odometry stale"
  wait_until "$FAST_LIO2_READY_TIMEOUT" "non-empty:/cloud_registered" pointcloud_nonempty /cloud_registered || startup_fail "/cloud_registered empty"
  wait_until "$FAST_LIO2_READY_TIMEOUT" "fresh:/cloud_registered" topic_fresh /cloud_registered || startup_fail "/cloud_registered stale"
  if [ "$(grep -Eic 'No Effective Points|\bnan\b|EKF.*diverg' "$WORKSPACE_DIR/logs/fast_lio2.log" 2>/dev/null || true)" -gt 20 ]; then
    LAST_OBSERVATION="sustained FAST-LIO2 invalid-point/divergence diagnostics"
    startup_fail "FAST-LIO2 health check failed"
  fi
  stage_pass

  RVIZ_CONFIG="$WORKSPACE_DIR/src/simenv_fast_lio2_integration/config/fast_lio2.rviz"
  if [ "$ENABLE_RVIZ" = "true" ] && [ -f "$RVIZ_CONFIG" ]; then
    launch_in_terminal "rviz" "rosrun rviz rviz -d ${RVIZ_CONFIG}"
  elif [ "$ENABLE_RVIZ" = "true" ]; then
    echo "[WARN] RViz config missing: $RVIZ_CONFIG" >&2
  fi
fi

if [ "$ENABLE_NAVIGATION" = "true" ]; then
  [ "$ENABLE_FAST_LIO2" = "true" ] || startup_fail "navigation requires FAST-LIO2"
  case "$NAV_MODE" in
    falco) START_DSV=false ;;
    dsv_falco) START_DSV=true ;;
    *) startup_fail "unsupported NAV_MODE=$NAV_MODE" ;;
  esac
  if [ "$NAV_AUTO_START_EXPLORATION" = "true" ] && { [ "$NAV_AUTO_ENABLE" != "true" ] || [ "$NAV_MODE" != "dsv_falco" ]; }; then
    startup_fail "auto exploration requires NAV_AUTO_ENABLE=true and NAV_MODE=dsv_falco"
  fi

  stage_enter STAGE_10_NAVIGATION_READY
  export NAV_MAX_LINEAR_X NAV_MAX_LINEAR_Y NAV_MAX_ANGULAR_Z NAV_COMMAND_TIMEOUT
  setsid roslaunch simenv_navigation_bringup single_floor_exploration.launch \
    start_falco:=true start_dsv:="$START_DSV" start_bridge:=true \
    > "$WORKSPACE_DIR/logs/navigation.log" 2>&1 &
  NAVIGATION_PID=$!
  echo "$NAVIGATION_PID" > "$WORKSPACE_DIR/logs/navigation.pid"
  for nav_node in /localPlanner /pathFollower /cmd_vel_bridge; do
    wait_until "$NAVIGATION_READY_TIMEOUT" "node:$nav_node" node_ready "$nav_node" || startup_fail "navigation node missing: $nav_node"
  done
  wait_until "$NAVIGATION_READY_TIMEOUT" "finite:/navigation/state_estimation" odom_finite /navigation/state_estimation || startup_fail "navigation odometry invalid"
  wait_until "$NAVIGATION_READY_TIMEOUT" "fresh:/navigation/state_estimation" topic_fresh /navigation/state_estimation || startup_fail "navigation odometry stale"
  wait_until "$NAVIGATION_READY_TIMEOUT" "non-empty:/navigation/registered_scan" pointcloud_nonempty /navigation/registered_scan || startup_fail "navigation scan invalid"
  wait_until "$NAVIGATION_READY_TIMEOUT" "fresh:/navigation/registered_scan" topic_fresh /navigation/registered_scan || startup_fail "navigation scan stale"
  state_stable /navigation/enabled false 2 || startup_fail "bridge gate initial state is not closed"
  if [ "$START_DSV" = "true" ]; then
    for nav_node in /navigation/dsvplanner /navigation/graph_planner /navigation/exploration; do
      wait_until "$NAVIGATION_READY_TIMEOUT" "node:$nav_node" node_ready "$nav_node" || startup_fail "DSV node missing: $nav_node"
    done
    wait_until "$NAVIGATION_READY_TIMEOUT" "service:/navigation/drrtPlannerSrv" service_ready /navigation/drrtPlannerSrv || startup_fail "DSV planner service missing"
    wait_until "$NAV_WAIT_TERRAIN_TIMEOUT" "non-empty:/navigation/terrain_map" pointcloud_nonempty /navigation/terrain_map || startup_fail "terrain map invalid"
  fi
  stage_pass "mode=$NAV_MODE gate=closed"

  stage_enter STAGE_11_EXPLORATION_STATE
  if [ "$NAV_AUTO_TROTTING" = "true" ]; then
    request_state /navigation/request_fsm_state std_msgs/Int8 4 /fsm/state_cmd "$STATE_TRANSITION_TIMEOUT" || startup_fail "Trotting transition failed"
    state_stable /fsm/state_cmd 4 3 || startup_fail "FSM=4 unstable"
  fi
  if [ "$NAV_AUTO_ENABLE" = "true" ]; then
    request_state /navigation/request_enabled std_msgs/Bool true /navigation/enabled "$STATE_TRANSITION_TIMEOUT" || startup_fail "navigation enable failed"
    state_stable /navigation/enabled true 3 || startup_fail "navigation enabled state unstable"
  fi
  if [ "$NAV_AUTO_START_EXPLORATION" = "true" ]; then
    request_state /navigation/request_exploring std_msgs/Bool true /navigation/start_exploring "$STATE_TRANSITION_TIMEOUT" || startup_fail "exploration enable failed"
    state_stable /navigation/start_exploring true 3 || startup_fail "exploration state unstable"
  fi
  stage_pass "trotting=$NAV_AUTO_TROTTING enabled=$NAV_AUTO_ENABLE exploring=$NAV_AUTO_START_EXPLORATION"
fi

if [ "$ENABLE_EXPLORATION_RECORDING" = "true" ]; then
  [ "$ENABLE_NAVIGATION" = "true" ] || startup_fail "recording requires navigation"
  stage_enter STAGE_12_RECORDER_READY
  [ -d "$EXPLORATION_OUTPUT_DIR" ] || mkdir -p "$EXPLORATION_OUTPUT_DIR"
  [ -w "$EXPLORATION_OUTPUT_DIR" ] || startup_fail "recorder output directory not writable"
  wait_until "$RECORDER_READY_TIMEOUT" "finite recorder odometry" odom_finite /Odometry || startup_fail "recorder odometry missing"
  wait_until "$RECORDER_READY_TIMEOUT" "registered cloud" pointcloud_nonempty /cloud_registered || startup_fail "recorder cloud missing"
  mkdir -p "$EXPLORATION_OUTPUT_DIR/logs"
  EXPLORATION_MINIMAL_MAP_VALIDATION="${EXPLORATION_MINIMAL_MAP_VALIDATION:-false}"
  EXPLORATION_STOP_AFTER_MAP_UPDATES="${EXPLORATION_STOP_AFTER_MAP_UPDATES:-3}"
  setsid roslaunch simenv_navigation_bringup exploration_recorder.launch \
    output_dir:="$EXPLORATION_OUTPUT_DIR" run_id:="$EXPLORATION_RUN_ID" \
    max_sim_time:="$EXPLORATION_MAX_SIM_TIME" finish_quiet_time:="$EXPLORATION_FINISH_QUIET_TIME" \
    map_stable_wait:="$EXPLORATION_MAP_STABLE_WAIT" minimal_map_validation:="$EXPLORATION_MINIMAL_MAP_VALIDATION" \
    stop_after_map_updates:="$EXPLORATION_STOP_AFTER_MAP_UPDATES" \
    > "$EXPLORATION_OUTPUT_DIR/logs/recorder.log" 2>&1 &
  RECORDER_PID=$!
  echo "$RECORDER_PID" > "$EXPLORATION_OUTPUT_DIR/logs/recorder.pid"
  wait_until "$RECORDER_READY_TIMEOUT" "recorder process" tracked_process_alive "$RECORDER_PID" recorder || startup_fail "recorder exited"
  stage_pass "mode=$NAV_MODE"
fi

stage_enter RUNTIME_ACTIVE
stage_pass
echo "Simulation startup command completed. Press Ctrl-C for safe shutdown."
while true; do
  sleep 1
done
