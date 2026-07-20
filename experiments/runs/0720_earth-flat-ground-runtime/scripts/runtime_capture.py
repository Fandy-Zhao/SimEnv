#!/usr/bin/env python3
"""Capture one runtime validation case from a running SimEnv/Gazebo session."""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from typing import Dict, List, Optional

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import runtime_metrics as metrics  # noqa: E402


FSM_FIXEDSTAND = 2
FSM_RL = 6


class TimingTail:
    def __init__(self, path: str):
        self.path = path
        self.rows: List[Dict[str, str]] = []
        self.latest: Optional[Dict[str, str]] = None

    def refresh(self) -> None:
        if not self.path or not os.path.exists(self.path) or os.path.getsize(self.path) == 0:
            return
        try:
            with open(self.path, newline="") as handle:
                rows = list(csv.DictReader(handle))
        except (OSError, csv.Error):
            return
        if rows:
            self.rows = rows
            self.latest = rows[-1]

    def latest_int(self, key: str, default: int = 0) -> int:
        self.refresh()
        for row in reversed(self.rows):
            try:
                value = row.get(key, "")
                if value != "":
                    return int(float(value))
            except (TypeError, ValueError):
                continue
        return default


class RuntimeCapture:
    def __init__(self, model_name: str):
        self.model_name = model_name
        self.model_names: List[str] = []
        self.pose_rows: List[Dict[str, float]] = []
        self.clock_rows: List[Dict[str, float]] = []
        self.joint_rows: List[Dict[str, float]] = []
        self.last_clock_wall: Optional[float] = None
        self.last_clock_sim: Optional[float] = None
        rospy.Subscriber("/clock", Clock, self._on_clock, queue_size=100)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_models, queue_size=100)
        rospy.Subscriber("/a1_gazebo/joint_states", JointState, self._on_joints, queue_size=50)

    def _on_clock(self, message: Clock) -> None:
        now_wall = time.monotonic()
        now_sim = message.clock.to_sec()
        row = {"wall_time": now_wall, "sim_time": now_sim}
        if self.last_clock_wall is not None and self.last_clock_sim is not None:
            wall_dt = max(now_wall - self.last_clock_wall, 1e-9)
            sim_dt = max(now_sim - self.last_clock_sim, 0.0)
            row["rtf"] = sim_dt / wall_dt
        self.last_clock_wall = now_wall
        self.last_clock_sim = now_sim
        self.clock_rows.append(row)

    def _on_models(self, message: ModelStates) -> None:
        self.model_names = list(message.name)
        if self.model_name not in message.name:
            return
        index = message.name.index(self.model_name)
        pose = message.pose[index]
        twist = message.twist[index]
        q = pose.orientation
        roll, pitch, yaw = metrics.quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.pose_rows.append(
            {
                "sim_time": rospy.Time.now().to_sec(),
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "qx": q.x,
                "qy": q.y,
                "qz": q.z,
                "qw": q.w,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "tilt_deg": metrics.quaternion_tilt_deg(q.x, q.y, q.z, q.w),
                "linear_x": twist.linear.x,
                "linear_y": twist.linear.y,
                "linear_z": twist.linear.z,
                "angular_x": twist.angular.x,
                "angular_y": twist.angular.y,
                "angular_z": twist.angular.z,
            }
        )

    def _on_joints(self, message: JointState) -> None:
        row: Dict[str, float] = {"sim_time": rospy.Time.now().to_sec()}
        for name, pos in zip(message.name, message.position):
            row["pos_" + name] = pos
        for name, vel in zip(message.name, message.velocity):
            row["vel_" + name] = vel
        self.joint_rows.append(row)


def write_csv(path: str, rows: List[Dict[str, float]]) -> None:
    fields = sorted({key for row in rows for key in row}) if rows else ["sim_time"]
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def wait_until(predicate, timeout_wall: float, poll: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout_wall
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(poll)
    return False


def wait_sim_duration(duration: float, wall_timeout: float, cmd_pub=None, cmd=None) -> bool:
    start = rospy.Time.now().to_sec()
    deadline = time.monotonic() + wall_timeout
    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if cmd_pub is not None and cmd is not None:
            cmd_pub.publish(cmd)
        if rospy.Time.now().to_sec() - start >= duration:
            return True
        rate.sleep()
    return False


def publish_state_until(pub, timing: TimingTail, state_id: int, timeout_wall: float) -> bool:
    message = Int8(data=state_id)
    deadline = time.monotonic() + timeout_wall
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        pub.publish(message)
        if timing.latest_int("fsm_state") == state_id:
            return True
        rate.sleep()
    return False


def twist(vx: float, vy: float, yaw_rate: float) -> Twist:
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = yaw_rate
    return msg


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-id", required=True)
    parser.add_argument("--case-kind", choices=("g0", "g1", "fixedstand", "rl"), required=True)
    parser.add_argument("--world-mode", choices=("earth", "competition"), default="earth")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--stop-duration", type=float, default=0.0)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--timing-csv", default="")
    parser.add_argument("--model-name", default="a1_gazebo")
    parser.add_argument("--wall-timeout", type=float, default=360.0)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("earth_runtime_capture_" + args.case_id.replace("-", "_"), anonymous=True)
    cap = RuntimeCapture(args.model_name)
    timing = TimingTail(args.timing_csv)
    state_pub = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=4)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    reasons: List[str] = []

    if not wait_until(lambda: cap.clock_rows and rospy.Time.now().to_sec() > 0.0, 60.0):
        reasons.append("NO_CLOCK")
    if not wait_until(lambda: cap.model_names, 60.0):
        reasons.append("NO_MODEL_STATES")

    start_index = len(cap.pose_rows)
    if args.case_kind == "g0":
        ok = wait_sim_duration(args.duration, args.wall_timeout)
    elif args.case_kind == "g1":
        if not wait_until(lambda: cap.pose_rows, 60.0):
            reasons.append("NO_ROBOT_POSE")
        ok = wait_sim_duration(args.duration, args.wall_timeout)
    else:
        target = FSM_FIXEDSTAND if args.case_kind == "fixedstand" else FSM_RL
        if not publish_state_until(state_pub, timing, FSM_FIXEDSTAND, 45.0):
            reasons.append("FIXEDSTAND_NOT_ENTERED")
        elif args.case_kind == "rl" and not publish_state_until(state_pub, timing, FSM_RL, 45.0):
            reasons.append("RL_NOT_ENTERED")
        start_index = len(cap.pose_rows)
        ok = wait_sim_duration(args.duration, args.wall_timeout, cmd_pub, twist(args.vx, args.vy, args.yaw_rate))
        if args.stop_duration > 0.0:
            ok = wait_sim_duration(args.stop_duration, args.wall_timeout, cmd_pub, twist(0.0, 0.0, 0.0)) and ok
        for _ in range(10):
            cmd_pub.publish(twist(0.0, 0.0, 0.0))
            time.sleep(0.02)
    if not ok:
        reasons.append("SIM_DURATION_INCOMPLETE")

    pose_rows = cap.pose_rows[start_index:]
    pose_summary = metrics.summarize_pose(pose_rows) if args.case_kind != "g0" else {"valid": True}
    rtf_summary = metrics.summarize_rtf(cap.clock_rows)
    models = sorted(cap.model_names)
    if args.world_mode == "earth" and ("platform_1" in models or "platform_2" in models):
        reasons.append("PLATFORM_STILL_PRESENT")
    final_fsm_state = timing.latest_int("fsm_state")
    if args.case_kind == "fixedstand" and final_fsm_state != FSM_FIXEDSTAND:
        reasons.append("FIXEDSTAND_NOT_HELD")
    if args.case_kind == "rl" and final_fsm_state != FSM_RL:
        reasons.append("RL_NOT_HELD")

    if args.case_kind == "fixedstand":
        verdict = metrics.classify_fixedstand(pose_summary)
    elif args.case_kind == "g0":
        verdict = "G0_PASS" if not reasons else "G0_FAIL_RUNTIME"
    elif args.case_kind == "g1":
        verdict = metrics.classify_fixedstand(pose_summary)
        verdict = "G1_PASS" if verdict == "PASS" and not reasons else "G1_FAIL_ATTITUDE"
    else:
        verdict = "PASS" if not reasons and pose_summary.get("valid") else "FAIL_RUNTIME"
        if pose_summary.get("valid") and float(pose_summary.get("max_tilt_deg", 0.0)) > 45.0:
            verdict = "FAIL_ATTITUDE"
        if args.vx > 0.0 and pose_summary.get("valid") and float(pose_summary.get("forward_displacement", 0.0)) <= 0.0:
            verdict = "FAIL_NO_FORWARD_RESPONSE"

    timing.refresh()
    result = {
        "schema_version": 1,
        "case_id": args.case_id,
        "case_kind": args.case_kind,
        "world_mode": args.world_mode,
        "requested_vx": args.vx,
        "requested_vy": args.vy,
        "requested_yaw_rate": args.yaw_rate,
        "requested_duration": args.duration,
        "stop_duration": args.stop_duration,
        "model_names": models,
        "platform_1_present": "platform_1" in models,
        "platform_2_present": "platform_2" in models,
        "ground_plane_present": "ground_plane" in models,
        "sun_present": "sun" in models,
        "invalid_reasons": sorted(set(reasons)),
        "final_fsm_state": final_fsm_state,
        "timing_rows": len(timing.rows),
        "verdict": verdict if not reasons or verdict.startswith("G0") or verdict.startswith("G1") else "INVALID",
        **pose_summary,
        **rtf_summary,
    }
    write_csv(os.path.join(args.output_dir, "pose_samples.csv"), pose_rows)
    write_csv(os.path.join(args.output_dir, "clock_rtf.csv"), cap.clock_rows)
    write_csv(os.path.join(args.output_dir, "joint_states.csv"), cap.joint_rows)
    with open(os.path.join(args.output_dir, "model_names.txt"), "w") as handle:
        handle.write("\n".join(models) + "\n")
    with open(os.path.join(args.output_dir, "metrics.json"), "w") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["verdict"] in ("G0_PASS", "G1_PASS", "PASS") or result["verdict"].startswith("FAIL") else 3


if __name__ == "__main__":
    raise SystemExit(main())
