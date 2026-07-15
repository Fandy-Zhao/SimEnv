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

SEED="${SEED:-}"
FLOOR_COUNT="${FLOOR_COUNT:-3}"
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
CONTROLLER_FOREGROUND="${CONTROLLER_FOREGROUND:-1}"
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
UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.004}"
GAZEBO_PHYSICS_MAX_STEP_SIZE="${GAZEBO_PHYSICS_MAX_STEP_SIZE:-0.002}"
GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE="${GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE:-500}"
GAZEBO_PHYSICS_ODE_ITERS="${GAZEBO_PHYSICS_ODE_ITERS:-40}"
GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL="${GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL:-5.0}"
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

echo "Cleaning up all leftover processes from previous runs..."

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

GENERATOR_SCRIPT="$WORKSPACE_DIR/src/building_obstacles/scripts/generate_competition_scene.py"
BUILDING_CONTROL_SCRIPT="$WORKSPACE_DIR/src/building_generator_classic/scripts/building_generator_classic_control"
UNITREE_GAZEBO_MODELS="$WORKSPACE_DIR/src/unitree_guide/unitree_ros/unitree_gazebo/models"
SCENE_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"
RESULTS_DIR="$WORKSPACE_DIR/results"
mkdir -p "$SCENE_OUTPUT_DIR" "$RESULTS_DIR" "$WORKSPACE_DIR/logs"

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
export COMPETITION_ROBOT_X="$ROBOT_X"
export COMPETITION_ROBOT_Y="$ROBOT_Y"
export COMPETITION_ROBOT_Z="$ROBOT_Z"
export COMPETITION_ROBOT_YAW="$ROBOT_YAW"
export UNITREE_CTRL_DT
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$SCENE_OUTPUT_DIR:$UNITREE_GAZEBO_MODELS"
export GAZEBO_PLUGIN_PATH="$WORKSPACE_DIR/devel/lib:${GAZEBO_PLUGIN_PATH:-}"

echo "=========================================="
echo "Competition scene is ready"
echo "  Workspace: $WORKSPACE_DIR"
echo "  World:   $BUILDING_WORLD_FILE"
echo "  Truth:   $RESULTS_DIR/danger_truth.json"
echo "  Manifest:$SCENE_OUTPUT_DIR/scene_manifest.json"
echo "  Result:  $RESULTS_DIR/detected_danger.json"
echo "  Sensor data: $ENABLE_SENSOR_DATA"
echo "  PointCloud2 converter: $ENABLE_POINTCLOUD_CONVERTER"
echo "  Ground truth topics: $ENABLE_GROUND_TRUTH"
echo "  Referee odom: $ENABLE_REFEREE_ODOM"
echo "  FAST-LIO2 mapping: $ENABLE_FAST_LIO2"
echo "  Gazebo starts paused: $PAUSED"
echo "  Auto unpause: $AUTO_UNPAUSE after ${AUTO_UNPAUSE_DELAY}s"
echo "  Gazebo physics: max_step=$GAZEBO_PHYSICS_MAX_STEP_SIZE update_rate=$GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE ode_iters=$GAZEBO_PHYSICS_ODE_ITERS"
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
# Controller startup (before FAST-LIO2 so the robot is standing for IMU init)
# ---------------------------------------------------------------------------
CTRL_IS_BACKGROUND=0
CTRL_PID=""
CTRL_WANTS_FOREGROUND=0
CTRL_TTY=""

# A background process started by a non-interactive shell receives /dev/null
# as stdin.  junior_ctrl's KeyBoard reads stdin directly, so explicitly bind
# it to the terminal that launched auto.sh instead of relying on Bash job
# control (fg is unavailable in a non-interactive script).
if [ -r /dev/tty ] && [ -w /dev/tty ]; then
  CTRL_TTY="/dev/tty"
elif [ "$START_CONTROLLER" = "1" ] && [ "$CONTROLLER_FOREGROUND" = "1" ]; then
  echo "WARNING: No controlling terminal is available; junior_ctrl keyboard input is disabled." >&2
  echo "  Run ./auto.sh from an interactive terminal, or use /fsm/state_cmd and /cmd_vel." >&2
fi

if [ "$START_CONTROLLER" = "1" ]; then
  if [ "$CONTROLLER_FOREGROUND" = "1" ] && [ "$ENABLE_FAST_LIO2" != "true" ]; then
    # No FAST-LIO2: simple foreground, blocks here.
    echo "Starting junior_ctrl controller in the foreground."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    echo "Use keyboard input: 2 = stand, 6 = RL mode."
    schedule_unpause_physics
    if [ -n "$CTRL_TTY" ]; then
      "$CONTROLLER_BIN" < "$CTRL_TTY"
    else
      "$CONTROLLER_BIN"
    fi
  else
    # Background first: script continues to auto-stabilise + FAST-LIO2.
    if [ "$CONTROLLER_FOREGROUND" = "1" ]; then
      CTRL_WANTS_FOREGROUND=1
    fi
    echo "Starting junior_ctrl controller in the background."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    if [ -n "$CTRL_TTY" ]; then
      "$CONTROLLER_BIN" \
        < "$CTRL_TTY" > "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>&1 &
    else
      "$CONTROLLER_BIN" \
        > "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>&1 &
    fi
    CTRL_PID=$!
    echo "$CTRL_PID" > "$WORKSPACE_DIR/logs/junior_ctrl.pid"
    CTRL_IS_BACKGROUND=1
    schedule_unpause_physics
  fi
else
  schedule_unpause_physics
fi

# ── FAST-LIO2 preflight: wait for upright robot ──
if [ "$ENABLE_FAST_LIO2" = "true" ] && [ "$CTRL_IS_BACKGROUND" = "1" ]; then
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
  rosrun simenv_fast_lio2_integration scan_to_pointcloud2.py \
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
fi

# ---------------------------------------------------------------------------
# Post-startup summary
# ---------------------------------------------------------------------------
echo "Simulation startup command completed."
echo "Publish geometry_msgs/Twist to /cmd_vel for velocity control (after Trotting/RL mode)."
echo "Use rostopic to switch FSM states:"
echo "  rostopic pub /fsm/state_cmd std_msgs/Int8 \"data: 2\"  # FixedStand"
echo "  rostopic pub /fsm/state_cmd std_msgs/Int8 \"data: 4\"  # Trotting"
echo "  rostopic pub /fsm/state_cmd std_msgs/Int8 \"data: 6\"  # RL"

# Keep the invoking terminal attached when foreground mode is requested.  The
# controller may be a background child during FAST-LIO2 startup, but it reads
# /dev/tty explicitly; waiting avoids unreliable `fg %1` job-control calls.
if [ "$CTRL_WANTS_FOREGROUND" = "1" ] && [ -n "$CTRL_PID" ]; then
  echo ""
  echo "Controller is reading this terminal (keyboard: 2=stand 4=trot 6=RL)."
  echo "Press Ctrl-C to stop auto.sh and junior_ctrl."
  wait "$CTRL_PID"
fi
