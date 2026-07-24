#!/usr/bin/env bash
set -euo pipefail

case_name="$1"
shift

run_dir="experiments/runs/0724_falco_a1_tuning"
case_dir="$run_dir/$case_name"
mkdir -p "$case_dir"

source devel/setup.bash

timeout 15s rostopic echo /navigation/state_estimation -n 1 > "$case_dir/odom.yaml" 2>&1 || \
timeout 15s rostopic echo /Odometry -n 1 > "$case_dir/odom.yaml"

python3 - "$case_dir/odom.yaml" "$case_dir/waypoint.txt" > "$case_dir/waypoint.yaml" <<'PY'
import math
import os
import pathlib
import re
import sys

odom_path = pathlib.Path(sys.argv[1])
out_path = pathlib.Path(sys.argv[2])
text = odom_path.read_text()

def get(pattern):
    match = re.search(pattern, text)
    if not match:
        raise SystemExit(f"missing pattern: {pattern}")
    return float(match.group(1))

frame = re.search(r'frame_id: "([^"]+)"', text).group(1)
x = get(r"position:\s*\n\s*x: ([^\n]+)")
y = get(r"position:\s*\n\s*x: [^\n]+\n\s*y: ([^\n]+)")
z = get(r"position:\s*\n\s*x: [^\n]+\n\s*y: [^\n]+\n\s*z: ([^\n]+)")
qx = get(r"orientation:\s*\n\s*x: ([^\n]+)")
qy = get(r"orientation:\s*\n\s*x: [^\n]+\n\s*y: ([^\n]+)")
qz = get(r"orientation:\s*\n\s*x: [^\n]+\n\s*y: [^\n]+\n\s*z: ([^\n]+)")
qw = get(r"orientation:\s*\n\s*x: [^\n]+\n\s*y: [^\n]+\n\s*z: [^\n]+\n\s*w: ([^\n]+)")
yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
forward = float(os.environ.get("WAYPOINT_FORWARD", "0.8"))
lateral = float(os.environ.get("WAYPOINT_LATERAL", "0.0"))
goal_x = x + forward * math.cos(yaw) - lateral * math.sin(yaw)
goal_y = y + forward * math.sin(yaw) + lateral * math.cos(yaw)
out_path.write_text(
    f"frame_id: {frame}\nodom_x: {x}\nodom_y: {y}\nodom_z: {z}\n"
    f"yaw: {yaw}\nforward: {forward}\nlateral: {lateral}\n"
    f"goal_x: {goal_x}\ngoal_y: {goal_y}\ngoal_z: {z}\n"
)
print(f"header: {{frame_id: {frame}}}")
print(f"point: {{x: {goal_x}, y: {goal_y}, z: {z}}}")
PY

roslaunch simenv_navigation_bringup runtime_real_data.launch \
  start_dsv:=false start_falco:=true start_bridge:=true \
  enable_diagnostics:=true diagnostic_throttle_sec:=0.5 "$@" \
  > "$case_dir/navigation.log" 2>&1 &
launch_pid=$!
trap 'kill "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true' EXIT

sleep 12
{
  echo "case: $case_name"
  echo "args: $*"
  echo "waypoint:"
  cat "$case_dir/waypoint.txt"
  rostopic pub /navigation/enabled std_msgs/Bool "data: false" -1
  rostopic pub /navigation/check_obstacle std_msgs/Bool "data: true" -1
  rostopic pub /navigation/way_point geometry_msgs/PointStamped "$(cat "$case_dir/waypoint.yaml")" -1
  sleep 45
  echo "clock:"
  timeout 8s rostopic echo /clock -n 1 || true
  echo "path:"
  timeout 12s rostopic echo /navigation/path -n 1 | sed -n '1,160p' || true
  echo "raw_cmd:"
  timeout 12s rostopic echo /navigation/falco/cmd_vel_stamped -n 1 | sed -n '1,80p' || true
  echo "gated_cmd:"
  (sleep 2; rostopic pub /navigation/enabled std_msgs/Bool "data: false" -1 >/dev/null 2>&1) &
  timeout 12s rostopic echo /cmd_vel -n 1 | sed -n '1,60p' || true
} > "$case_dir/case.log" 2>&1

grep -h "falco_diag" "$case_dir/navigation.log" ~/.ros/log/latest/localPlanner-*.log 2>/dev/null \
  | tail -40 > "$case_dir/falco_diag.log" || true

python3 - "$case_name" "$case_dir" "$run_dir/parameter_matrix.csv" <<'PY'
import csv
import pathlib
import re
import sys

case = sys.argv[1]
case_dir = pathlib.Path(sys.argv[2])
matrix_path = pathlib.Path(sys.argv[3])
diag_lines = [line for line in (case_dir / "falco_diag.log").read_text(errors="ignore").splitlines() if "falco_diag" in line]
diag = diag_lines[-1] if diag_lines else ""
case_log = (case_dir / "case.log").read_text(errors="ignore")

def field(name, default=""):
    match = re.search(rf"{name}=([^\s]+)", diag)
    return match.group(1) if match else default

def yaml_value(section, name):
    marker = section + ":"
    idx = case_log.find(marker)
    if idx < 0:
        return ""
    sub = case_log[idx:]
    match = re.search(rf"{name}:\s*([-+0-9.eE]+)", sub)
    return match.group(1) if match else ""

path_pose_count = field("published_poses")
raw_linear_x = yaml_value("raw_cmd", "x")
raw_angular_z = yaml_value("angular", "z")
row = {
    "case": case,
    "args": re.search(r"^args:[ \t]*(.*)$", case_log, re.M).group(1) if re.search(r"^args:[ \t]*(.*)$", case_log, re.M) else "",
    "raw": field("raw"),
    "adjacent": field("adjacent"),
    "height_filtered": field("height_filtered"),
    "center02": field("center02"),
    "center03": field("center03"),
    "center05": field("center05"),
    "candidate_paths": field("candidate_paths"),
    "free_paths": field("free_paths"),
    "selected_group": field("selected_group"),
    "selected_rot": field("selected_rot"),
    "published_poses": path_pose_count,
    "raw_linear_x": raw_linear_x,
    "raw_angular_z": raw_angular_z,
    "diag": diag,
}
write_header = not matrix_path.exists()
with matrix_path.open("a", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=list(row))
    if write_header:
        writer.writeheader()
    writer.writerow(row)
PY
