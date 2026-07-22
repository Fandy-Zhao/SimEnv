#!/usr/bin/env python3
"""Capture FixedStand-to-RL transition evidence for an already running epoch."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import time
from typing import Any, Dict, List, Optional

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Int8
from unitree_legged_msgs.msg import MotorCmd


FSM_FIXEDSTAND = 2
FSM_RL = 6
JOINT_ORDER = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]


def tilt_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    z_axis_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    return math.degrees(math.acos(max(-1.0, min(1.0, z_axis_z))))


def median(values: List[float]) -> Optional[float]:
    vals = sorted(v for v in values if math.isfinite(v))
    return statistics.median(vals) if vals else None


class Capture:
    def __init__(self, model_name: str):
        self.model_name = model_name
        self.clock_rows: List[Dict[str, float]] = []
        self.rows: List[Dict[str, Any]] = []
        self.model_names: List[str] = []
        self.latest_model: Dict[str, Any] = {}
        self.latest_joint: Dict[str, float] = {}
        self.latest_imu: Dict[str, float] = {}
        self.latest_cmd: Dict[str, float] = {"cmd_vx": 0.0, "cmd_vy": 0.0, "cmd_yaw": 0.0}
        self.latest_motor_cmd: Dict[str, float] = {}
        self.last_clock_wall: Optional[float] = None
        self.last_clock_sim: Optional[float] = None
        self.latest_rtf: Optional[float] = None

        rospy.Subscriber("/clock", Clock, self.on_clock, queue_size=200)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_model, queue_size=100)
        rospy.Subscriber("/a1_gazebo/joint_states", JointState, self.on_joint, queue_size=50)
        rospy.Subscriber("/trunk_imu", Imu, self.on_imu, queue_size=100)
        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=50)
        for joint in JOINT_ORDER:
            topic = f"/a1_gazebo/{joint}_controller/command"
            rospy.Subscriber(topic, MotorCmd, self.on_motor_cmd, callback_args=joint, queue_size=10)

    def on_clock(self, msg: Clock) -> None:
        wall = time.monotonic()
        sim = msg.clock.to_sec()
        row = {"wall_time": wall, "sim_time": sim}
        if self.last_clock_wall is not None and self.last_clock_sim is not None:
            wall_dt = max(wall - self.last_clock_wall, 1e-9)
            sim_dt = max(sim - self.last_clock_sim, 0.0)
            self.latest_rtf = sim_dt / wall_dt
            row["rtf"] = self.latest_rtf
        self.last_clock_wall = wall
        self.last_clock_sim = sim
        self.clock_rows.append(row)

    def on_model(self, msg: ModelStates) -> None:
        self.model_names = list(msg.name)
        if self.model_name not in msg.name:
            return
        idx = msg.name.index(self.model_name)
        pose = msg.pose[idx]
        twist = msg.twist[idx]
        q = pose.orientation
        self.latest_model = {
            "base_x": pose.position.x,
            "base_y": pose.position.y,
            "base_z": pose.position.z,
            "base_qx": q.x,
            "base_qy": q.y,
            "base_qz": q.z,
            "base_qw": q.w,
            "tilt_deg": tilt_deg(q.x, q.y, q.z, q.w),
            "base_vx": twist.linear.x,
            "base_vy": twist.linear.y,
            "base_vz": twist.linear.z,
            "base_wx": twist.angular.x,
            "base_wy": twist.angular.y,
            "base_wz": twist.angular.z,
        }

    def on_joint(self, msg: JointState) -> None:
        latest: Dict[str, float] = {}
        for name, value in zip(msg.name, msg.position):
            latest[f"pos_{name}"] = value
        for name, value in zip(msg.name, msg.velocity):
            latest[f"vel_{name}"] = value
        self.latest_joint = latest

    def on_imu(self, msg: Imu) -> None:
        self.latest_imu = {
            "imu_qw": msg.orientation.w,
            "imu_qx": msg.orientation.x,
            "imu_qy": msg.orientation.y,
            "imu_qz": msg.orientation.z,
            "imu_wx": msg.angular_velocity.x,
            "imu_wy": msg.angular_velocity.y,
            "imu_wz": msg.angular_velocity.z,
        }

    def on_cmd(self, msg: Twist) -> None:
        self.latest_cmd = {
            "cmd_vx": msg.linear.x,
            "cmd_vy": msg.linear.y,
            "cmd_yaw": msg.angular.z,
        }

    def on_motor_cmd(self, msg: MotorCmd, joint: str) -> None:
        self.latest_motor_cmd[f"target_{joint}"] = msg.q
        self.latest_motor_cmd[f"kp_{joint}"] = msg.Kp
        self.latest_motor_cmd[f"kd_{joint}"] = msg.Kd
        self.latest_motor_cmd[f"mode_{joint}"] = msg.mode

    def sample(self, phase: str, switch_sim: Optional[float]) -> None:
        if not self.latest_model:
            return
        sim = rospy.Time.now().to_sec()
        row: Dict[str, Any] = {
            "sim_time": sim,
            "rel_to_rl_switch": "" if switch_sim is None else sim - switch_sim,
            "phase": phase,
            "rtf": self.latest_rtf if self.latest_rtf is not None else "",
        }
        row.update(self.latest_cmd)
        row.update(self.latest_model)
        row.update(self.latest_joint)
        row.update(self.latest_imu)
        row.update(self.latest_motor_cmd)
        self.rows.append(row)


def publish_state(pub: rospy.Publisher, state: int, duration_wall: float) -> None:
    msg = Int8(data=state)
    rate = rospy.Rate(10)
    deadline = time.monotonic() + duration_wall
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        pub.publish(msg)
        rate.sleep()


def publish_zero(cmd_pub: rospy.Publisher) -> None:
    cmd_pub.publish(Twist())


def publish_state_sampled(
    state_pub: rospy.Publisher,
    cmd_pub: rospy.Publisher,
    cap: Capture,
    state: int,
    phase: str,
    switch_sim: float,
    duration_wall: float,
) -> None:
    msg = Int8(data=state)
    rate = rospy.Rate(50)
    deadline = time.monotonic() + duration_wall
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        state_pub.publish(msg)
        publish_zero(cmd_pub)
        cap.sample(phase, switch_sim)
        rate.sleep()


def write_csv(path: str, rows: List[Dict[str, Any]]) -> None:
    fields = sorted({key for row in rows for key in row})
    preferred = [
        "phase", "sim_time", "rel_to_rl_switch", "cmd_vx", "cmd_vy", "cmd_yaw",
        "base_x", "base_y", "base_z", "tilt_deg", "base_vx", "base_vy",
        "base_wx", "base_wy", "base_wz", "imu_wx", "imu_wy", "imu_wz", "rtf",
    ]
    ordered = [f for f in preferred if f in fields] + [f for f in fields if f not in preferred]
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=ordered, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows: List[Dict[str, Any]], switch_sim: float) -> Dict[str, Any]:
    post = [r for r in rows if isinstance(r.get("rel_to_rl_switch"), float) and r["rel_to_rl_switch"] >= 0.0]
    after_grace = [r for r in post if r["rel_to_rl_switch"] >= 1.0]
    heights = [float(r["base_z"]) for r in post if r.get("base_z") not in ("", None)]
    tilts = [float(r["tilt_deg"]) for r in post if r.get("tilt_deg") not in ("", None)]
    targets = [
        abs(float(v))
        for r in post[:10]
        for k, v in r.items()
        if k.startswith("target_") and v not in ("", None)
    ]
    return {
        "switch_sim_time": switch_sim,
        "sample_count": len(rows),
        "post_sample_count": len(post),
        "post_duration_sim": (post[-1]["sim_time"] - post[0]["sim_time"]) if len(post) >= 2 else 0.0,
        "post_min_base_height": min(heights) if heights else None,
        "post_max_tilt_deg": max(tilts) if tilts else None,
        "post_grace_min_base_height": min((float(r["base_z"]) for r in after_grace), default=None),
        "post_grace_max_tilt_deg": max((float(r["tilt_deg"]) for r in after_grace), default=None),
        "first_10_sample_abs_target_max": max(targets) if targets else None,
        "rtf_median": median([float(r["rtf"]) for r in rows if r.get("rtf") not in ("", None)]),
        "fell": bool(heights and min(heights) < 0.18),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--model-name", default="a1_gazebo")
    parser.add_argument("--fixedstand-hold-sim", type=float, default=4.0)
    parser.add_argument("--post-rl-sim", type=float, default=3.5)
    parser.add_argument("--label", default="baseline_transition_zero")
    parser.add_argument("--wall-timeout", type=float, default=120.0)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("rl_transition_capture", anonymous=True)
    cap = Capture(args.model_name)
    state_pub = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=4)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=20)

    deadline = time.monotonic() + args.wall_timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if cap.clock_rows and cap.latest_model and cap.latest_joint and cap.latest_imu:
            break
        time.sleep(0.05)
    if not (cap.clock_rows and cap.latest_model and cap.latest_joint and cap.latest_imu):
        raise RuntimeError("required ROS streams did not become ready")

    publish_state(state_pub, FSM_FIXEDSTAND, 2.0)
    start_fixed = rospy.Time.now().to_sec()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - start_fixed < args.fixedstand_hold_sim:
        publish_zero(cmd_pub)
        cap.sample("fixedstand_hold", None)
        rate.sleep()

    # Keep only the last one simulated second before RL plus the post window.
    pre_rows = list(cap.rows)
    pre_cutoff = rospy.Time.now().to_sec() - 1.0
    cap.rows = [r for r in pre_rows if float(r["sim_time"]) >= pre_cutoff]

    switch_sim = rospy.Time.now().to_sec()
    publish_state_sampled(state_pub, cmd_pub, cap, FSM_RL, "rl_zero", switch_sim, 1.0)
    start_post = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - start_post < args.post_rl_sim:
        publish_zero(cmd_pub)
        cap.sample("rl_zero", switch_sim)
        rate.sleep()

    csv_path = os.path.join(args.output_dir, f"{args.label}.csv")
    json_path = os.path.join(args.output_dir, f"{args.label}_summary.json")
    write_csv(csv_path, cap.rows)
    summary = summarize(cap.rows, switch_sim)
    with open(json_path, "w") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
