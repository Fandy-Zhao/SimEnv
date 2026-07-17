#!/usr/bin/env bash
set -u
exec 9>/tmp/simenv-gazebo.lock; flock 9
WT=/home/zzf/search_ws/SimEnv_worktrees/stage2
TROT=/home/zzf/search_ws/SimEnv_worktrees/trot-rl
EXT=/home/zzf/search_ws/SimEnv
OUT="$WT/experiments/runs/0717_fastlio2-stage2"
source /opt/ros/noetic/setup.bash; source "$TROT/devel/setup.bash"
export ROS_PACKAGE_PATH="$WT/src:$TROT/src:$EXT/src:$ROS_PACKAGE_PATH"
export GAZEBO_PLUGIN_PATH="$TROT/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="$TROT/devel/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="$TROT/devel:${CMAKE_PREFIX_PATH:-}"
cleanup(){ kill "${CAP:-}" "${FAST:-}" "${ADAPTER:-}" "${CTRL:-}" "${AUTO:-}" 2>/dev/null||true; wait "${CAP:-}" "${FAST:-}" "${ADAPTER:-}" "${CTRL:-}" "${AUTO:-}" 2>/dev/null||true; }
trap cleanup EXIT INT TERM
cd "$WT"
env SEED=20260717 FLOOR_COUNT=1 GUI=false ENABLE_RVIZ=0 TERMINAL_BACKEND=direct ENABLE_FAST_LIO2=0 ENABLE_POINTCLOUD_CONVERTER=0 START_CONTROLLER=1 GAZEBO_PLUGIN_PATH="$GAZEBO_PLUGIN_PATH" LD_LIBRARY_PATH="$LD_LIBRARY_PATH" CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH" ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH" ./auto.sh >"$OUT/overlay_auto.log" 2>&1 & AUTO=$!
rm -f "$OUT/overlay_upright.txt" "$OUT/runtime_progress.json" "$OUT/runtime_metrics.json"
for _ in $(seq 1 120); do rosparam list >/dev/null 2>&1&&break;sleep .25;done
for _ in $(seq 1 120);do rostopic pub -1 /fsm/state_cmd std_msgs/Int8 'data: 2' >/dev/null 2>&1||true; Z=$(timeout 10 rostopic echo -n1 /trunk_imu/linear_acceleration/z 2>/dev/null|head -1||true); if awk -v z="$Z" 'BEGIN{exit !(z>=8&&z<=11)}';then echo "trunk_imu_z=$Z">"$OUT/overlay_upright.txt";UP=1;break;fi;sleep 1;done
test "${UP:-0}" = 1||exit 23
for _ in $(seq 1 240); do rostopic type /scan 2>/dev/null|grep -q sensor_msgs/PointCloud&&break;sleep 1;done
rostopic type /scan >"$OUT/overlay_scan_type.txt" 2>&1||exit 20
/usr/bin/python3 "$WT/src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py" __name:=stage2_scan_adapter >"$OUT/overlay_adapter.log" 2>&1 & ADAPTER=$!
roslaunch "$WT/src/simenv_fast_lio2_integration/launch/simenv_fast_lio2_mapping.launch" enable_adapter:=false rviz:=false save_pcd:=false >"$OUT/overlay_fastlio.log" 2>&1 & FAST=$!
for _ in $(seq 1 300);do if rostopic type /state_estimation 2>/dev/null|grep -q nav_msgs/Odometry&&rostopic type /registered_scan 2>/dev/null|grep -q sensor_msgs/PointCloud2;then READY=1;break;fi;sleep 1;done
test "${READY:-0}" = 1||exit 24
rostopic type /state_estimation >"$OUT/overlay_state_type.txt";rostopic type /registered_scan >"$OUT/overlay_cloud_type.txt"
/usr/bin/python3 "$OUT/capture_runtime.py" 150 "$OUT/runtime_metrics.json" "$OUT/runtime_progress.json" >"$OUT/overlay_capture.log" 2>&1 & CAP=$!; wait "$CAP"
