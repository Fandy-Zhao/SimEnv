#!/usr/bin/env bash
set -euo pipefail

# Kill existing FAST-LIO2 nodes
rosnode kill /laserMapping 2>/dev/null || true
rosnode kill /scan_to_pointcloud2 2>/dev/null || true
sleep 2

# Kill any orphan processes
kill -9 $(pgrep -f "fastlio_mapping") 2>/dev/null || true
kill -9 $(pgrep -f "scan_to_pointcloud2.py") 2>/dev/null || true
sleep 1

# Clear old params that were in wrong namespace
rosparam delete /laserMapping 2>/dev/null || true

# Re-launch with fixed config
source /opt/ros/noetic/setup.bash
source /home/zzf/search_ws/SimEnv/devel/setup.bash
roslaunch simenv_fast_lio2_integration simenv_fast_lio2_mapping.launch
