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
ENABLE_POINTCLOUD_CONVERTER="$(as_ros_bool "${ENABLE_POINTCLOUD_CONVERTER:-1}")"
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
NAV_AUTO_TROTTING="$(as_ros_bool "${NAV_AUTO_TROTTING:-0}")"
NAV_AUTO_ENABLE="$(as_ros_bool "${NAV_AUTO_ENABLE:-0}")"
NAV_AUTO_START_EXPLORATION="$(as_ros_bool "${NAV_AUTO_START_EXPLORATION:-0}")"
NAV_WAIT_ODOM_TIMEOUT="${NAV_WAIT_ODOM_TIMEOUT:-60}"
NAV_WAIT_CLOUD_TIMEOUT="${NAV_WAIT_CLOUD_TIMEOUT:-60}"
NAV_WAIT_TERRAIN_TIMEOUT="${NAV_WAIT_TERRAIN_TIMEOUT:-60}"
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

schedule_unpause_physics() {
  if [ "$AUTO_UNPAUSE" != "true" ]; then
    return
  fi

  (
    sleep "$AUTO_UNPAUSE_DELAY"
    for _ in $(seq 1 40); do
      if rosservice list 2>/dev/null | grep -q '^/gazebo/unpause_physics$'; then
        rosservice call /gazebo/unpause_physics >/dev/null 2>&1 || true
        exit 0
      fi
      sleep 0.25
    done
  ) &
}

# ---------------------------------------------------------------------------
# GAZEBO_FINAL_UNPAUSE — call AFTER all nodes, controllers, and navigation
# are fully initialised.  Verifies that /clock is advancing before returning.
# ---------------------------------------------------------------------------
gazebo_final_unpause() {
  local unpause_max_retries="${GAZEBO_UNPAUSE_MAX_RETRIES:-3}"
  local clock_stale_timeout="${GAZEBO_CLOCK_STALE_TIMEOUT:-3.0}"
  local attempt

  # ── Unpause ──
  for attempt in $(seq 1 "$unpause_max_retries"); do
    if rosservice list 2>/dev/null | grep -q '^/gazebo/unpause_physics$'; then
      rosservice call /gazebo/unpause_physics >/dev/null 2>&1 || true
      echo "[GAZEBO_FINAL_UNPAUSE] unpause called (attempt $attempt/$unpause_max_retries)"
    else
      echo "[GAZEBO_FINAL_UNPAUSE] /gazebo/unpause_physics not available (attempt $attempt)" >&2
      sleep 1.0
      continue
    fi

    # ── Verify /clock is advancing ──
    sleep 0.5
    local clock1_sec clock1_nsec clock2_sec clock2_nsec clock1 clock2
    read -r clock1_sec clock1_nsec < <(
      rostopic echo /clock -n 1 2>/dev/null \
        | grep -E "^[[:space:]]*(secs|nsecs):" | head -2 \
        | awk '{sec=$2; getline; nsec=$2; print sec, nsec}'
    ) || true
    sleep 1.0
    read -r clock2_sec clock2_nsec < <(
      rostopic echo /clock -n 1 2>/dev/null \
        | grep -E "^[[:space:]]*(secs|nsecs):" | head -2 \
        | awk '{sec=$2; getline; nsec=$2; print sec, nsec}'
    ) || true

    clock1="${clock1_sec:-?}.${clock1_nsec:-?}"
    clock2="${clock2_sec:-?}.${clock2_nsec:-?}"
    if [ -n "$clock1_sec" ] && [ -n "$clock1_nsec" ] && [ -n "$clock2_sec" ] && [ -n "$clock2_nsec" ]; then
      if [ "$clock2_sec" -gt "$clock1_sec" ] 2>/dev/null || \
         { [ "$clock2_sec" -eq "$clock1_sec" ] 2>/dev/null && \
           [ "$clock2_nsec" -gt "$clock1_nsec" ] 2>/dev/null; }; then
        echo "[GAZEBO_FINAL_UNPAUSE] PASS: /clock advancing ($clock1 → $clock2)"
        return 0
      fi
    fi

    echo "[GAZEBO_FINAL_UNPAUSE] WARNING: /clock appears stalled (clock1=$clock1 clock2=$clock2)" >&2
  done

  echo "[GAZEBO_FINAL_UNPAUSE] FAILED: /clock not advancing after $unpause_max_retries attempts" >&2
  return 1
}

# ---------------------------------------------------------------------------
# Helper: wait for a ROS topic to publish at least two consecutive messages
# with incrementing timestamps (proves the publisher is alive, not a stale
# latched message).
# Usage: wait_for_topic <topic> [timeout_sec]
# Returns: 0 on success, 1 on timeout.
# ---------------------------------------------------------------------------
wait_for_topic() {
  local topic="$1"
  local timeout_sec="${2:-30}"
  local start_time prev_sec prev_nsec cur_sec cur_nsec

  start_time="$(date +%s)"
  prev_sec="" prev_nsec=""

  while true; do
    cur_sec=""
    cur_nsec=""
    read -r cur_sec cur_nsec < <(
      timeout --kill-after=2s 8 rostopic echo -n 1 "$topic" 2>/dev/null \
        | grep -E "^[[:space:]]*(secs|nsecs):" | head -2 \
        | awk '{sec=$2; getline; nsec=$2; print sec, nsec}'
    ) 2>/dev/null || true

    if [ -n "$cur_sec" ] && [ -n "$cur_nsec" ]; then
      if [ -z "$prev_sec" ]; then
        # First valid message captured
        prev_sec="$cur_sec"
        prev_nsec="$cur_nsec"
      else
        # Check timestamp is incrementing (proves live publisher)
        if [ "$cur_sec" -gt "$prev_sec" ] 2>/dev/null || \
           { [ "$cur_sec" -eq "$prev_sec" ] 2>/dev/null && \
             [ "$cur_nsec" -gt "$prev_nsec" ] 2>/dev/null; }; then
          echo "[READY] topic: $topic (timestamps incrementing)"
          return 0
        fi
        # Timestamp not incrementing: reset and retry
        prev_sec="$cur_sec"
        prev_nsec="$cur_nsec"
      fi
    fi

    if [ $(( $(date +%s) - start_time )) -ge "$timeout_sec" ]; then
      echo "[ERROR] timed out waiting for topic: $topic" >&2
      return 1
    fi

    sleep 0.5
  done
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

if [ "$SKIP_GLOBAL_PROCESS_CLEANUP" = "true" ]; then
  echo "Skipping global process cleanup (SKIP_GLOBAL_PROCESS_CLEANUP=true)."
else
  echo "Cleaning up all leftover processes from previous runs..."
  cleanup_tmux_sessions

# ---- Gazebo & ROS core ----
pkill -9 -f "gzserver"     2>/dev/null || true
pkill -9 -f "gzclient"     2>/dev/null || true
pkill -9 -f "gazebo"       2>/dev/null || true
pkill -9 -f "rosmaster"    2>/dev/null || true
pkill -9 -f "rosout"       2>/dev/null || true

# ---- SimEnv launch & control ----
pkill -9 -f "roslaunch.*multi_floor_gazeboSim"       2>/dev/null || true
pkill -9 -f "roslaunch.*simenv_fast_lio2_mapping"    2>/dev/null || true
pkill -9 -f "building_generator_classic_control"      2>/dev/null || true
pkill -9 -f "generate_competition_scene"              2>/dev/null || true

# ---- Unitree controller ----
pkill -9 -f "junior_ctrl"          2>/dev/null || true
pkill -9 -f "unitree_gazebo_servo" 2>/dev/null || true
pkill -9 -f "virtual_joy"          2>/dev/null || true

# ---- FAST-LIO2 ----
pkill -9 -f "fastlio_mapping"       2>/dev/null || true
pkill -9 -f "scan_to_pointcloud2"   2>/dev/null || true
pkill -9 -f "laserMapping"          2>/dev/null || true

# ---- Navigation bringup ----
pkill -9 -f "roslaunch.*simenv_navigation_bringup" 2>/dev/null || true
pkill -9 -f "cmd_vel_bridge"         2>/dev/null || true
pkill -9 -f "localPlanner"           2>/dev/null || true
pkill -9 -f "pathFollower"           2>/dev/null || true
pkill -9 -f "exploration"           2>/dev/null || true
pkill -9 -f "dsvplanner"            2>/dev/null || true
pkill -9 -f "graph_planner"         2>/dev/null || true
pkill -9 -f "navigation_boundary"   2>/dev/null || true
pkill -9 -f "registered_cloud_to_terrain_map" 2>/dev/null || true
pkill -9 -f "navigation_registered_scan_relay" 2>/dev/null || true
pkill -9 -f "navigation_state_estimation_relay" 2>/dev/null || true

# ---- Sensor / state bridge nodes ----
pkill -9 -f "pointcloud2livox"      2>/dev/null || true
pkill -9 -f "state_from_gazebo"     2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "controller_spawner"    2>/dev/null || true

# ---- Stale rostopic processes ----
pkill -9 -f "rostopic.*/cmd_vel"    2>/dev/null || true
pkill -9 -f "rostopic.*echo"        2>/dev/null || true
fi

# Give the OS a moment to reclaim ports and shared memory
sleep 2

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
echo "  PointCloud2 converter: $ENABLE_POINTCLOUD_CONVERTER"
echo "  Ground truth topics: $ENABLE_GROUND_TRUTH"
echo "  Referee odom: $ENABLE_REFEREE_ODOM"
echo "  FAST-LIO2 mapping: $ENABLE_FAST_LIO2"
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
echo "  Auto unpause: $AUTO_UNPAUSE after ${AUTO_UNPAUSE_DELAY}s"
echo "  Gazebo physics:"
echo "    max_step_size:            $GAZEBO_PHYSICS_MAX_STEP_SIZE"
echo "    real_time_update_rate:    $GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE"
echo "    ode_iters:                $GAZEBO_PHYSICS_ODE_ITERS"
echo "    contact_max_correcting_vel: $GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL"
echo "    theoretical target RTF:   $_GAZEBO_PHYSICS_RTF_PRODUCT"
echo "  Gazebo plugin path: $GAZEBO_PLUGIN_PATH"
echo "=========================================="

if [ "$START_VIRTUAL_JOY" = "1" ]; then
  echo "Starting virtual joystick. This may require uinput permissions."
  rosrun unitree_guide virtual_joy.py > "$WORKSPACE_DIR/logs/virtual_joy.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/virtual_joy.pid"
fi

echo "Launching Gazebo, Unitree A1 model, sensors, and ROS interfaces..."
roslaunch unitree_guide multi_floor_gazeboSim.launch \
  gui:="$GUI" \
  paused:="$PAUSED" \
  user_debug:=False \
  rname:=a1 \
  robot_x:="$ROBOT_X" \
  robot_y:="$ROBOT_Y" \
  robot_z:="$ROBOT_Z" \
  robot_yaw:="$ROBOT_YAW" \
  enable_sensor_data:="$ENABLE_SENSOR_DATA" \
  enable_referee_odom:="$ENABLE_REFEREE_ODOM" \
  enable_ground_truth:="$ENABLE_GROUND_TRUTH" \
  enable_foot_force_visual:="$ENABLE_FOOT_FORCE_VISUAL" \
  enable_joy_node:="$ENABLE_JOY_NODE" \
  enable_pointcloud_converter:="$ENABLE_POINTCLOUD_CONVERTER" \
  pointcloud_use_ground_truth_odom:="$POINTCLOUD_USE_GROUND_TRUTH_ODOM" \
  > "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$WORKSPACE_DIR/logs/competition_gazebo.pid"
sleep 25
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo "roslaunch exited during startup. Last log lines:" >&2
  tail -n 80 "$WORKSPACE_DIR/logs/competition_gazebo.log" >&2
  exit 1
fi

if [ "$START_BUILDING_CONTROL" = "1" ]; then
  echo "Starting building door/elevator control service..."
  python3 "$BUILDING_CONTROL_SCRIPT" \
    --door-config "$SCENE_OUTPUT_DIR/door_config.yaml" \
    --elevator-config "$SCENE_OUTPUT_DIR/elevator_config.yaml" \
    > "$WORKSPACE_DIR/logs/building_control.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/building_control.pid"
fi

# ---------------------------------------------------------------------------
# Controller startup — always in a dedicated terminal so keyboard input works.
# ---------------------------------------------------------------------------
if [ "$START_CONTROLLER" = "1" ]; then
  echo "Starting junior_ctrl in a dedicated terminal..."
  echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
  echo "Keyboard input in the controller terminal: 2=stand (4=trot, 6=RL need Torch build)."
  if [ "$TIMING_DIAGNOSTICS_ENABLED" = "true" ]; then
    rosparam set /timing_diagnostics_enabled true
    rosparam set /timing_diagnostics_path "$TIMING_DIAGNOSTICS_PATH"
  fi
  launch_in_terminal "junior_ctrl" "$CONTROLLER_BIN"
  sleep 2
  schedule_unpause_physics
else
  schedule_unpause_physics
fi

# ── Command FixedStand (always, before any FAST-LIO2 / navigation) ──
if [ "$START_CONTROLLER" = "1" ]; then
  # Wait for the controller node and its /fsm/state_cmd subscriber.
  sleep 3

  # Apply any user-requested extra delay before commanding FixedStand.
  FAST_LIO2_DELAY="${FAST_LIO2_DELAY:-0}"
  if [ "$FAST_LIO2_DELAY" -gt 0 ]; then
    echo "Waiting additional ${FAST_LIO2_DELAY}s (FAST_LIO2_DELAY)..."
    sleep "$FAST_LIO2_DELAY"
  fi

  # Wait for /fsm/state_cmd subscriber before publishing (avoid lost message).
  echo "Waiting for /fsm/state_cmd subscriber..."
  for i in $(seq 1 15); do
    if rostopic info /fsm/state_cmd 2>/dev/null | grep -q 'Subscribers.*http'; then
      echo "  /fsm/state_cmd has subscriber(s)"
      break
    fi
    sleep 1
  done

  # Auto-command FixedStand: rostopic pub … std_msgs/Int8 "data: 2"
  echo "Commanding FixedStand via /fsm/state_cmd..."
  rostopic pub /fsm/state_cmd std_msgs/Int8 "data: 2" -1 2>/dev/null || true
fi

# ── FAST-LIO2 preflight: wait for upright robot ──
if [ "$ENABLE_FAST_LIO2" = "true" ] && [ "$START_CONTROLLER" = "1" ]; then

  # Wait for the IMU to report gravity aligned with Z (≥ 9 m/s²).
  # This confirms the robot is upright before FAST-LIO2 initialises.
  echo "Waiting for IMU stabilisation (robot upright)..."
  IMU_STABLE=0
  for i in $(seq 1 20); do
    sleep 0.5
    IMU_Z=$(timeout 10 rostopic echo /trunk_imu/linear_acceleration -n 1 2>/dev/null \
      | grep "z:" | head -1 | awk '{print $2}')
    if [ -n "$IMU_Z" ]; then
      IMU_Z_INT=$(echo "$IMU_Z" | cut -d. -f1)
      if [ "$IMU_Z_INT" -ge 9 ] 2>/dev/null; then
        echo "  IMU stabilised: linear_acceleration.z ≈ ${IMU_Z} m/s²"
        IMU_STABLE=1
        break
      fi
    fi
    echo "  waiting for upright pose ... IMU z = ${IMU_Z:-N/A}"
  done
  if [ "$IMU_STABLE" != "1" ]; then
    echo "WARNING: IMU did not stabilise within 20 s." >&2
    echo "  FAST-LIO2 may initialise with incorrect gravity, causing Z drift." >&2
  fi
fi

# ---------------------------------------------------------------------------
# FAST-LIO2 (after the robot is confirmed standing)
# ---------------------------------------------------------------------------
if [ "$ENABLE_FAST_LIO2" = "true" ]; then
  if [ "$START_CONTROLLER" != "1" ]; then
    echo "WARNING: ENABLE_FAST_LIO2=1 but START_CONTROLLER=$START_CONTROLLER" >&2
    echo "  The robot will NOT be standing when FAST-LIO2 initialises." >&2
    echo "  FAST-LIO2 needs a stationary, upright robot for correct EKF convergence." >&2
    echo "  Either set START_CONTROLLER=1 or start the controller manually (FixedStand)." >&2
    echo "" >&2
  fi

  echo "Starting FAST-LIO2 mapping (scan adapter + fastlio_mapping)..."
  # Run with Noetic's system Python even when a catkin wrapper was generated
  # from an active Conda environment.
  /usr/bin/python3 "$WORKSPACE_DIR/src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py" \
    > "$WORKSPACE_DIR/logs/scan_adapter.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/scan_adapter.pid"
  sleep 2
  # enable_adapter:=false avoids a duplicate scan_to_pointcloud2 node
  # (auto.sh already started one above).
  roslaunch simenv_fast_lio2_integration simenv_fast_lio2_mapping.launch \
    enable_adapter:=false \
    > "$WORKSPACE_DIR/logs/fast_lio2.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/fast_lio2.pid"
  echo "FAST-LIO2 mapping launched in background (logs: logs/fast_lio2.log)"

  # ── RVIZ ──
  RVIZ_CONFIG="$WORKSPACE_DIR/src/simenv_fast_lio2_integration/config/fast_lio2.rviz"
  if [ "$ENABLE_RVIZ" = "true" ] && [ -f "$RVIZ_CONFIG" ]; then
    echo "Starting rviz in a dedicated terminal..."
    launch_in_terminal "rviz" "rosrun rviz rviz -d ${RVIZ_CONFIG}"
  elif [ "$ENABLE_RVIZ" = "true" ]; then
    echo "WARNING: rviz config not found at $RVIZ_CONFIG" >&2
  fi
fi

# ---------------------------------------------------------------------------
# Navigation bringup -- DSV + FALCO exploration stack.
# Launched after FAST-LIO2 is confirmed ready.  Nodes are started but
# motion is NOT enabled until the user explicitly commands it.
# ---------------------------------------------------------------------------
if [ "$ENABLE_NAVIGATION" = "true" ]; then
  if [ "$ENABLE_FAST_LIO2" != "true" ]; then
    echo "[ERROR] Navigation requires ENABLE_FAST_LIO2=1 (currently ENABLE_FAST_LIO2=$ENABLE_FAST_LIO2)." >&2
    exit 1
  fi

  case "$NAV_MODE" in
    falco)     START_DSV="false" ;;
    dsv_falco) START_DSV="true"  ;;
    *)
      echo "[ERROR] Unsupported NAV_MODE='$NAV_MODE'. Allowed: falco, dsv_falco" >&2
      exit 1
      ;;
  esac

  if [ "$NAV_AUTO_START_EXPLORATION" = "true" ]; then
    if [ "$NAV_AUTO_ENABLE" != "true" ] || [ "$NAV_MODE" != "dsv_falco" ]; then
      echo "[ERROR] NAV_AUTO_START_EXPLORATION=true requires:" >&2
      echo "  NAV_AUTO_ENABLE=true (currently $NAV_AUTO_ENABLE)" >&2
      echo "  NAV_MODE=dsv_falco (currently $NAV_MODE)" >&2
      exit 1
    fi
    if [ "$NAV_AUTO_TROTTING" != "true" ]; then
      echo "NAV_AUTO_START_EXPLORATION=true with NAV_AUTO_TROTTING=false:"
      echo "  DSV exploration will plan goals but robot will NOT auto-trot."
      echo "  To enable auto-trotting, set NAV_AUTO_TROTTING=true."
    fi
  fi

  echo "Waiting for navigation prerequisites..."
  wait_for_topic /Odometry "$NAV_WAIT_ODOM_TIMEOUT" || exit 1
  wait_for_topic /cloud_registered "$NAV_WAIT_CLOUD_TIMEOUT" || exit 1

  # Export speed limits so navigation_bridge.launch picks them up via $(optenv).
  export NAV_MAX_LINEAR_X
  export NAV_MAX_LINEAR_Y
  export NAV_MAX_ANGULAR_Z
  export NAV_COMMAND_TIMEOUT

  # Navigation state supervisor — latched state owner that survives
  # bridge / navigation sub-stack restarts.  Launched independently so
  # that killing the navigation roslaunch does not lose user-commanded
  # state (enabled, exploring, fsm_state).
  echo "Launching navigation state supervisor..."
  rosrun simenv_navigation_bridge nav_state_supervisor.py \
    > "$WORKSPACE_DIR/logs/nav_state_supervisor.log" 2>&1 &
  SUPERVISOR_PID=$!
  echo "$SUPERVISOR_PID" > "$WORKSPACE_DIR/logs/nav_state_supervisor.pid"
  echo "Navigation state supervisor launched (pid=$SUPERVISOR_PID)"

  echo "Launching navigation bringup (mode=$NAV_MODE, dsv=$START_DSV)..."
  roslaunch simenv_navigation_bringup single_floor_exploration.launch \
    start_falco:=true \
    start_dsv:="$START_DSV" \
    start_bridge:=true \
    > "$WORKSPACE_DIR/logs/navigation.log" 2>&1 &

  NAVIGATION_PID=$!
  echo "$NAVIGATION_PID" > "$WORKSPACE_DIR/logs/navigation.pid"
  echo "Navigation bringup launched (pid=$NAVIGATION_PID, logs: logs/navigation.log)"

  sleep 6

  echo "Setting default safe navigation state (enabled=false, start_exploring=false)..."
  rostopic pub /navigation/enabled       std_msgs/Bool  "data: false" -1 2>/dev/null || true
  rostopic pub /navigation/start_exploring std_msgs/Bool "data: false" -1 2>/dev/null || true
  echo "Navigation nodes are running but motion is DISABLED."
  echo "  Enable motion manually:"
  echo "    rostopic pub /fsm/state_cmd std_msgs/Int8 \"data: 4\" -1"
  echo "    rostopic pub /navigation/enabled std_msgs/Bool \"data: true\" -1"
  echo "    rostopic pub /navigation/start_exploring std_msgs/Bool \"data: true\" -1"

  if [ "$NAV_AUTO_TROTTING" = "true" ]; then
    sleep 2
    echo "NAV_AUTO_TROTTING=true: commanding Trotting via /fsm/state_cmd..."
    rostopic pub /fsm/state_cmd std_msgs/Int8 "data: 4" -1 2>/dev/null || true
  fi
  if [ "$NAV_AUTO_ENABLE" = "true" ]; then
    sleep 2
    echo "NAV_AUTO_ENABLE=true: publishing /navigation/enabled=true..."
    rostopic pub /navigation/enabled std_msgs/Bool "data: true" -1 2>/dev/null || true
  fi
  if [ "$NAV_AUTO_START_EXPLORATION" = "true" ]; then
    sleep 2
    echo "NAV_AUTO_START_EXPLORATION=true: publishing /navigation/start_exploring=true..."
    rostopic pub /navigation/start_exploring std_msgs/Bool "data: true" -1 2>/dev/null || true
  fi
fi

# ---------------------------------------------------------------------------
# Post-startup summary
# ---------------------------------------------------------------------------
echo "Simulation startup command completed."
if [ "$ENABLE_NAVIGATION" = "true" ]; then
  echo ""
  echo "  ---- Navigation Control ----"
  echo "  Nodes: DSV=$START_DSV  FALCO=true  Bridge=true"
  echo "  State: nodes alive, motion DISABLED (safe)"
  echo ""
  echo "  To enable exploration:"
  echo "    rostopic pub /fsm/state_cmd std_msgs/Int8 \"data: 4\" -1"
  echo "    rostopic pub /navigation/enabled std_msgs/Bool \"data: true\" -1"
  echo "    rostopic pub /navigation/start_exploring std_msgs/Bool \"data: true\" -1"
  echo ""
  echo "  To stop motion immediately:"
  echo "    rostopic pub /navigation/enabled std_msgs/Bool \"data: false\" -1"
else
  echo "Publish geometry_msgs/Twist to /cmd_vel for velocity control (Trotting/RL mode only;"
  echo "  Trotting and RL require a Torch-enabled build: set UNITREE_ENABLE_TORCH_POLICY=ON)."
  echo "Use rostopic to switch FSM states:"
  echo "  rostopic pub /fsm/state_cmd std_msgs/Int8 \"data: 2\"  # FixedStand"
  echo "  rostopic pub /fsm/state_cmd std_msgs/Int8 \"data: 4\"  # Trotting   (needs Torch)"
  echo "  rostopic pub /fsm/state_cmd std_msgs/Int8 \"data: 6\"  # RL         (needs Torch)"
fi

# Keep the script alive so the user can read the summary and press Ctrl-C to
# stop all processes.  The controller and rviz run in their own terminals.
echo ""
echo "=============================================="
echo "  auto.sh startup complete."
echo "  The controller and rviz are in separate ${TERMINAL_BACKEND} sessions."
if [ "$TERMINAL_BACKEND" = "tmux" ]; then
  echo "  Reattach if a GUI terminal closes: tmux attach-session -t ${TMUX_SESSION_PREFIX}-junior_ctrl"
  echo "                                 tmux attach-session -t ${TMUX_SESSION_PREFIX}-rviz"
fi
echo "  Press Ctrl-C in THIS terminal to stop all ROS processes."
echo "=============================================="

# ── GAZEBO_FINAL_UNPAUSE: ensure simulation clock is running ──
gazebo_final_unpause || true

# Cleanup trap: kill ROS processes when the user presses Ctrl-C.
cleanup() {
  echo ""
  echo "Shutting down..."

  # ── Disable navigation motion before killing nodes ──
  if [ "${ENABLE_NAVIGATION:-false}" = "true" ]; then
    rostopic pub /navigation/enabled std_msgs/Bool "data: false" -1 2>/dev/null || true
    sleep 0.5
  fi

  # ── Navigation bringup ──
  pkill -9 -f "roslaunch.*simenv_navigation_bringup" 2>/dev/null || true
  pkill -9 -f "nav_state_supervisor"  2>/dev/null || true
  pkill -9 -f "cmd_vel_bridge"         2>/dev/null || true
  pkill -9 -f "localPlanner"           2>/dev/null || true
  pkill -9 -f "pathFollower"           2>/dev/null || true
  pkill -9 -f "exploration"           2>/dev/null || true
  pkill -9 -f "dsvplanner"            2>/dev/null || true
  pkill -9 -f "graph_planner"         2>/dev/null || true
  pkill -9 -f "navigation_boundary"   2>/dev/null || true
  pkill -9 -f "registered_cloud_to_terrain_map" 2>/dev/null || true
  pkill -9 -f "navigation_registered_scan_relay" 2>/dev/null || true
  pkill -9 -f "navigation_state_estimation_relay" 2>/dev/null || true

  # ── Gazebo and core infrastructure ──
  pkill -9 -f "gzserver"     2>/dev/null || true
  pkill -9 -f "gzclient"     2>/dev/null || true
  pkill -9 -f "gazebo"       2>/dev/null || true
  pkill -9 -f "rosmaster"    2>/dev/null || true
  pkill -9 -f "rosout"       2>/dev/null || true
  pkill -9 -f "junior_ctrl"  2>/dev/null || true
  pkill -9 -f "fastlio_mapping" 2>/dev/null || true
  pkill -9 -f "laserMapping" 2>/dev/null || true
  pkill -9 -f "scan_to_pointcloud2" 2>/dev/null || true
  pkill -9 -f "building_generator_classic_control" 2>/dev/null || true
  pkill -9 -f "rviz"         2>/dev/null || true
  cleanup_tmux_sessions
  echo "Cleanup complete."
  exit 0
}
trap cleanup INT TERM

# Wait indefinitely — the user stops the simulation with Ctrl-C.
while true; do
  sleep 1
done
