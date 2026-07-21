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
#   PHYSICS_PROFILE=normal    Validated balance of performance and contact fidelity.
#                             Earth A1 average RTF >= 0.8 with reliable contact.
#   PHYSICS_PROFILE=fidelity  Original high-resolution configuration (0.0002 s /
#                             5000 Hz / ODE 50).  Low RTF; for special verification.
#
# Explicit single-parameter overrides (GAZEBO_PHYSICS_MAX_STEP_SIZE, etc.)
# take precedence over the profile default.  Unknown profile values are rejected.
# ---------------------------------------------------------------------------
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
echo "  Physics profile: $PHYSICS_PROFILE"
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
echo "  Building controller: $START_BUILDING_CONTROL"
echo "  Virtual joystick: $START_VIRTUAL_JOY"
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
  launch_in_terminal "junior_ctrl" "$CONTROLLER_BIN"
  sleep 2
  schedule_unpause_physics
else
  schedule_unpause_physics
fi

# ── FAST-LIO2 preflight: wait for upright robot ──
if [ "$ENABLE_FAST_LIO2" = "true" ] && [ "$START_CONTROLLER" = "1" ]; then
  # Wait for the controller node and its /fsm/state_cmd subscriber.
  sleep 3

  # Apply any user-requested extra delay before commanding FixedStand.
  FAST_LIO2_DELAY="${FAST_LIO2_DELAY:-0}"
  if [ "$FAST_LIO2_DELAY" -gt 0 ]; then
    echo "Waiting additional ${FAST_LIO2_DELAY}s (FAST_LIO2_DELAY)..."
    sleep "$FAST_LIO2_DELAY"
  fi

  # Auto-command FixedStand: rostopic pub … std_msgs/Int8 "data: 2"
  echo "Commanding FixedStand via /fsm/state_cmd..."
  rostopic pub /fsm/state_cmd std_msgs/Int8 "data: 2" -1 2>/dev/null || true

  # Wait for the IMU to report gravity aligned with Z (≥ 9 m/s²).
  # This confirms the robot is upright before FAST-LIO2 initialises.
  echo "Waiting for IMU stabilisation (robot upright)..."
  IMU_STABLE=0
  for i in $(seq 1 40); do
    sleep 0.5
    IMU_Z=$(rostopic echo /trunk_imu/linear_acceleration -n 1 2>/dev/null \
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
# Post-startup summary
# ---------------------------------------------------------------------------
echo "Simulation startup command completed."
echo "Publish geometry_msgs/Twist to /cmd_vel for velocity control (Trotting/RL mode only;"
echo "  Trotting and RL require a Torch-enabled build: set UNITREE_ENABLE_TORCH_POLICY=ON)."
echo "Use rostopic to switch FSM states:"
echo "  rostopic pub /fsm/state_cmd std_msgs/Int8 \"data: 2\"  # FixedStand"
echo "  rostopic pub /fsm/state_cmd std_msgs/Int8 \"data: 4\"  # Trotting   (needs Torch)"
echo "  rostopic pub /fsm/state_cmd std_msgs/Int8 \"data: 6\"  # RL         (needs Torch)"

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

# Cleanup trap: kill ROS processes when the user presses Ctrl-C.
cleanup() {
  echo ""
  echo "Shutting down..."
  pkill -9 -f "gzserver"     2>/dev/null || true
  pkill -9 -f "gzclient"     2>/dev/null || true
  pkill -9 -f "gazebo"       2>/dev/null || true
  pkill -9 -f "rosmaster"    2>/dev/null || true
  pkill -9 -f "rosout"       2>/dev/null || true
  pkill -9 -f "junior_ctrl"  2>/dev/null || true
  pkill -9 -f "fastlio_mapping" 2>/dev/null || true
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
