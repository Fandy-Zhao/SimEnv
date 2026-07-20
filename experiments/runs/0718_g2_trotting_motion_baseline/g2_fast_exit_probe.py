#!/usr/bin/env python3
"""Run one Gate A fast-exit probe inside an existing ROS/Gazebo runtime."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

FSM_FIXEDSTAND = 2
FSM_TROTTING = 4
WAVE_ALL = 2
FALL_HEIGHT_M = 0.12
FOOT_TOPICS = {
    "FR": "/visual/FR_foot_contact/the_force",
    "FL": "/visual/FL_foot_contact/the_force",
    "RR": "/visual/RR_foot_contact/the_force",
    "RL": "/visual/RL_foot_contact/the_force",
}


@dataclass
class EventLog:
    rows: List[Dict[str, object]] = field(default_factory=list)

    def add(self, event: str, detail: str = "") -> None:
        row = {
            "wall_time": time.time(),
            "sim_time": rospy.Time.now().to_sec() if not rospy.is_shutdown() else 0.0,
            "event": event,
            "detail": detail,
        }
        self.rows.append(row)
        rospy.loginfo("G2 fast-exit event: %s %s", event, detail)

    def write(self, path: str) -> None:
        write_csv(path, self.rows, ["wall_time", "sim_time", "event", "detail"])


class TimingTail:
    def __init__(self, path: str):
        self.path = path
        self.latest: Optional[Dict[str, str]] = None
        self.rows: List[Dict[str, str]] = []

    def refresh(self) -> None:
        if not os.path.exists(self.path) or os.path.getsize(self.path) == 0:
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
        if not self.latest:
            return default
        try:
            return int(float(self.latest.get(key, default)))
        except (TypeError, ValueError):
            return default


class ProbeCapture:
    def __init__(self, model_name: str):
        self.model_name = model_name
        self.truth_rows: List[Dict[str, float]] = []
        self.joint_rows: List[Dict[str, object]] = []
        self.foot_rows: List[Dict[str, object]] = []
        self.foot_sequence = {leg: 0 for leg in FOOT_TOPICS}
        self.last_truth_write_time = -1.0
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_models, queue_size=50)
        rospy.Subscriber("/a1_gazebo/joint_states", JointState, self._on_joint, queue_size=50)
        for leg, topic in FOOT_TOPICS.items():
            rospy.Subscriber(topic, WrenchStamped, self._make_foot_cb(leg), queue_size=50)

    def _on_models(self, message: ModelStates) -> None:
        if self.model_name not in message.name:
            return
        now = rospy.Time.now().to_sec()
        if self.last_truth_write_time >= 0.0 and now - self.last_truth_write_time < 0.01:
            return
        self.last_truth_write_time = now
        index = message.name.index(self.model_name)
        pose = message.pose[index]
        twist = message.twist[index]
        roll, pitch, yaw = quaternion_to_euler(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        self.truth_rows.append(
            {
                "sim_time": now,
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "world_vx": twist.linear.x,
                "world_vy": twist.linear.y,
                "world_vz": twist.linear.z,
                "world_wz": twist.angular.z,
            }
        )

    def _on_joint(self, message: JointState) -> None:
        now = rospy.Time.now().to_sec()
        self.joint_rows.append(
            {
                "sim_time": now,
                "name": ";".join(message.name),
                "position": ";".join(str(value) for value in message.position),
                "velocity": ";".join(str(value) for value in message.velocity),
                "effort": ";".join(str(value) for value in message.effort),
            }
        )

    def _make_foot_cb(self, leg: str):
        def callback(message: WrenchStamped) -> None:
            self.foot_sequence[leg] += 1
            force = message.wrench.force
            self.foot_rows.append(
                {
                    "sim_time": rospy.Time.now().to_sec(),
                    "leg": leg,
                    "sequence": self.foot_sequence[leg],
                    "fx": force.x,
                    "fy": force.y,
                    "fz": force.z,
                    "force_norm": math.sqrt(force.x * force.x + force.y * force.y + force.z * force.z),
                }
            )

        return callback

    def write(self, output_dir: str) -> None:
        write_csv(os.path.join(output_dir, "ground_truth.csv"), self.truth_rows)
        write_csv(os.path.join(output_dir, "joint_state.csv"), self.joint_rows)
        write_csv(os.path.join(output_dir, "foot_force.csv"), self.foot_rows)


def quaternion_to_euler(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def write_csv(path: str, rows: List[Dict[str, object]], fields: Optional[List[str]] = None) -> None:
    fieldnames = fields or sorted({key for row in rows for key in row}) or ["sim_time"]
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def wait_until(predicate, timeout_wall: float, poll: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout_wall
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(poll)
    return False


def wait_sim_duration(duration: float, wall_timeout: float, publisher=None, message=None) -> bool:
    start = rospy.Time.now().to_sec()
    deadline = time.monotonic() + wall_timeout
    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if publisher is not None and message is not None:
            publisher.publish(message)
        if rospy.Time.now().to_sec() - start >= duration:
            return True
        rate.sleep()
    return False


def publish_state_until(pub, data: int, timing: TimingTail, target_state: int, timeout_wall: float) -> bool:
    message = Int8(data=data)
    deadline = time.monotonic() + timeout_wall
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        pub.publish(message)
        if timing.latest_int("fsm_state") == target_state:
            return True
        rate.sleep()
    return False


def publish_cmd_for(pub, vx: float, duration_sim: float, wall_timeout: float) -> bool:
    message = Twist()
    message.linear.x = vx
    start = rospy.Time.now().to_sec()
    deadline = time.monotonic() + wall_timeout
    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        pub.publish(message)
        if rospy.Time.now().to_sec() - start >= duration_sim:
            return True
        rate.sleep()
    return False


def finite_rows(rows: List[Dict[str, object]]) -> bool:
    for row in rows:
        for value in row.values():
            if isinstance(value, float) and not math.isfinite(value):
                return False
    return True


def safe_int(value, default: int = 0) -> int:
    try:
        if value in (None, ""):
            return default
        return int(float(value))
    except (TypeError, ValueError):
        return default


def summarize_probe(args, capture: ProbeCapture, timing: TimingTail, reasons: List[str]) -> Dict[str, object]:
    timing.refresh()
    heights = [row["z"] for row in capture.truth_rows]
    rolls = [abs(row["roll"]) for row in capture.truth_rows]
    pitches = [abs(row["pitch"]) for row in capture.truth_rows]
    min_height = min(heights) if heights else 0.0
    max_tilt_rad = max(rolls + pitches) if rolls or pitches else 0.0
    wave_all = any(safe_int(row.get("wave_status"), -1) == WAVE_ALL for row in timing.rows)
    gait_cycles = [
        safe_int(row.get("gait_cycle_sequence"), 0)
        for row in timing.rows
        if row.get("gait_cycle_sequence", "") != ""
    ]
    if not capture.truth_rows or not timing.rows:
        reasons.append("DATA_INCOMPLETE")
    if min_height and min_height < FALL_HEIGHT_M:
        reasons.append("FALL_DETECTED")
    if not finite_rows(capture.truth_rows):
        reasons.append("TRUTH_NONFINITE")
    if not finite_rows(capture.joint_rows):
        reasons.append("JOINT_NONFINITE")
    return {
        "schema_version": 1,
        "probe_id": args.probe_id,
        "probe_mode": args.probe_mode,
        "command_vx": args.command_vx,
        "result": "PASS" if not reasons else "FAIL",
        "reasons": sorted(set(reasons)),
        "truth_samples": len(capture.truth_rows),
        "joint_samples": len(capture.joint_rows),
        "foot_samples": len(capture.foot_rows),
        "foot_sequences": capture.foot_sequence,
        "timing_rows": len(timing.rows),
        "min_model_height": min_height,
        "max_abs_tilt_rad": max_tilt_rad,
        "wave_all_entered": wave_all,
        "gait_cycle_start": min(gait_cycles) if gait_cycles else 0,
        "gait_cycle_end": max(gait_cycles) if gait_cycles else 0,
        "final_fsm_state": timing.latest_int("fsm_state"),
        "final_wave_status": timing.latest_int("wave_status", -1),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe-id", required=True)
    parser.add_argument("--probe-mode", choices=["p0_fixedstand", "p1_trotting_zero", "p2_trotting_vx"], required=True)
    parser.add_argument("--command-vx", type=float, default=0.0)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--timing-csv", required=True)
    parser.add_argument("--model-name", default="a1_gazebo")
    parser.add_argument("--wall-timeout", type=float, default=180.0)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("g2_fast_exit_%s" % args.probe_id.replace("-", "_"), anonymous=True)
    events = EventLog()
    capture = ProbeCapture(args.model_name)
    timing = TimingTail(args.timing_csv)
    state_pub = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=4)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    reasons: List[str] = []

    events.add("PROBE_STARTED", args.probe_mode)
    if not wait_until(lambda: capture.truth_rows and rospy.Time.now().to_sec() > 0.0, 60.0):
        reasons.append("ROBOT_SPAWN_TIMEOUT")
    else:
        events.add("ROBOT_SPAWNED", args.model_name)

    if not wait_until(lambda: all(seq > 0 for seq in capture.foot_sequence.values()), 30.0):
        reasons.append("CONTACT_NOT_READY")
    else:
        events.add("CONTACT_READY", json.dumps(capture.foot_sequence, sort_keys=True))

    if not publish_state_until(state_pub, 2, timing, FSM_FIXEDSTAND, 30.0):
        reasons.append("FIXEDSTAND_NOT_ENTERED")
    else:
        events.add("FIXEDSTAND_ENTERED")
        if not wait_sim_duration(3.0, 90.0):
            reasons.append("FIXEDSTAND_HOLD_TIMEOUT")
        else:
            events.add("FIXEDSTAND_HOLD_COMPLETE", "3.0s sim-time")

    if args.probe_mode != "p0_fixedstand":
        if not publish_state_until(state_pub, 4, timing, FSM_TROTTING, 30.0):
            reasons.append("TROTTING_NOT_ENTERED")
        else:
            events.add("TROTTING_ENTERED")
            if args.probe_mode == "p1_trotting_zero":
                ok = publish_cmd_for(cmd_pub, 0.0, 2.0, args.wall_timeout)
                events.add("ZERO_COMMAND_WINDOW", json.dumps({"completed": ok}))
                if not ok:
                    reasons.append("ZERO_COMMAND_TIMEOUT")
            else:
                ok = publish_cmd_for(cmd_pub, args.command_vx, 5.0, args.wall_timeout)
                events.add("LOW_SPEED_COMMAND_WINDOW", json.dumps({"completed": ok, "vx": args.command_vx}))
                if not ok:
                    reasons.append("LOW_SPEED_COMMAND_TIMEOUT")

    for _ in range(10):
        cmd_pub.publish(Twist())
        time.sleep(0.02)

    status = summarize_probe(args, capture, timing, reasons)
    capture.write(args.output_dir)
    events.write(os.path.join(args.output_dir, "events.csv"))
    with open(os.path.join(args.output_dir, "probe_status.json"), "w") as handle:
        json.dump(status, handle, indent=2, sort_keys=True)
    print(json.dumps(status, indent=2, sort_keys=True))
    return 0 if status["result"] == "PASS" else 3


if __name__ == "__main__":
    raise SystemExit(main())
