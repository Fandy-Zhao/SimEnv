#!/usr/bin/env python3
"""Run one G2-B Trotting trial inside an already-started ROS/Gazebo runtime."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import g2_metrics  # noqa: E402


FSM_PASSIVE = 1
FSM_FIXEDSTAND = 2
FSM_TROTTING = 4
WAVE_ALL = 2
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
        rowsafe = {
            "wall_time": time.time(),
            "sim_time": rospy.Time.now().to_sec() if not rospy.is_shutdown() else 0.0,
            "event": event,
            "detail": detail,
        }
        self.rows.append(rowsafe)
        rospy.loginfo("G2 event: %s %s", event, detail)

    def write(self, path: str) -> None:
        with open(path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["wall_time", "sim_time", "event", "detail"])
            writer.writeheader()
            writer.writerows(self.rows)


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

    def latest_float(self, key: str, default: float = 0.0) -> float:
        self.refresh()
        if not self.latest:
            return default
        try:
            return float(self.latest.get(key, default))
        except (TypeError, ValueError):
            return default


class TrialCapture:
    def __init__(self, model_name: str):
        self.model_name = model_name
        self.truth_rows: List[Dict[str, float]] = []
        self.joint_rows: List[Dict[str, object]] = []
        self.foot_rows: List[Dict[str, object]] = []
        self.foot_sequence = {leg: 0 for leg in FOOT_TOPICS}
        self.foot_last_stamp = {leg: 0.0 for leg in FOOT_TOPICS}
        self.last_truth_sim_time = 0.0
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
        roll, pitch, yaw = g2_metrics.quaternion_to_euler(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        body_vx, body_vy = g2_metrics.world_to_body_velocity(twist.linear.x, twist.linear.y, yaw)
        row = {
            "sim_time": now,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "world_vx": twist.linear.x,
            "world_vy": twist.linear.y,
            "world_vz": twist.linear.z,
            "body_vx": body_vx,
            "body_vy": body_vy,
            "body_vz": twist.linear.z,
            "body_wz": twist.angular.z,
            "world_wz": twist.angular.z,
        }
        self.truth_rows.append(row)
        self.last_truth_sim_time = now

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
            now = rospy.Time.now().to_sec()
            self.foot_last_stamp[leg] = now
            force = message.wrench.force
            self.foot_rows.append(
                {
                    "sim_time": now,
                    "leg": leg,
                    "sequence": self.foot_sequence[leg],
                    "fx": force.x,
                    "fy": force.y,
                    "fz": force.z,
                    "force_norm": math.sqrt(force.x * force.x + force.y * force.y + force.z * force.z),
                }
            )

        return callback

    def write_csvs(self, output_dir: str) -> None:
        write_csv(os.path.join(output_dir, "ground_truth.csv"), self.truth_rows)
        write_csv(os.path.join(output_dir, "joint_state.csv"), self.joint_rows)
        write_csv(os.path.join(output_dir, "foot_force.csv"), self.foot_rows)


def write_csv(path: str, rows: List[Dict[str, object]]) -> None:
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


def publish_cmd_for(pub, vx: float, duration_sim: float, wall_timeout: float) -> Optional[Dict[str, float]]:
    message = Twist()
    message.linear.x = vx
    message.linear.y = 0.0
    message.angular.z = 0.0
    start = rospy.Time.now().to_sec()
    ok = wait_sim_duration(duration_sim, wall_timeout, pub, message)
    end = rospy.Time.now().to_sec()
    return {"sim_start": start, "sim_end": end, "completed": ok}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--command-vx", type=float, required=True)
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--timing-csv", required=True)
    parser.add_argument("--model-name", default="a1_gazebo")
    parser.add_argument("--wall-timeout", type=float, default=180.0)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("g2_capture_%s" % args.trial_id.replace("-", "_"), anonymous=True)
    events = EventLog()
    capture = TrialCapture(args.model_name)
    timing = TimingTail(args.timing_csv)
    state_pub = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=4)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    reasons: List[str] = []

    events.add("TRIAL_STARTED", args.trial_id)
    if not wait_until(lambda: capture.truth_rows and rospy.Time.now().to_sec() > 0.0, 60.0):
        reasons.append("DATA_INCOMPLETE")
    else:
        events.add("ROBOT_SPAWNED", args.model_name)

    if not wait_until(lambda: all(seq > 0 for seq in capture.foot_sequence.values()), 30.0):
        reasons.append("CONTACT_NOT_READY")
    else:
        events.add("CONTACT_READY", json.dumps(capture.foot_sequence, sort_keys=True))

    if not publish_state_until(state_pub, 2, timing, FSM_FIXEDSTAND, 30.0):
        reasons.append("FSM_TRANSITION_FAILED")
    else:
        events.add("FIXED_STAND_ENTERED")
        wait_sim_duration(1.0, 20.0)

    if not publish_state_until(state_pub, 4, timing, FSM_TROTTING, 30.0):
        reasons.append("FSM_TRANSITION_FAILED")
    else:
        events.add("TROTTING_ENTERED")

    wave_start_seq = timing.latest_int("gait_cycle_sequence")
    if not wait_until(lambda: timing.latest_int("wave_status") == WAVE_ALL, 30.0):
        reasons.append("WAVE_ALL_NOT_REACHED")
    else:
        events.add("WAVE_ALL_ENTERED")

    if not wait_until(lambda: timing.latest_int("gait_cycle_sequence") > wave_start_seq, 10.0):
        reasons.append("GAIT_NOT_ADVANCING")

    zero_ready = publish_cmd_for(cmd_pub, 0.0, 1.5, 40.0)
    active = publish_cmd_for(cmd_pub, args.command_vx, 5.0, args.wall_timeout)
    events.add("COMMAND_STARTED", json.dumps(active, sort_keys=True))
    stopped = publish_cmd_for(cmd_pub, 0.0, 2.0, args.wall_timeout)
    events.add("COMMAND_ZEROED", json.dumps(stopped, sort_keys=True))
    for _ in range(10):
        cmd_pub.publish(Twist())
        time.sleep(0.02)

    if not active or not active["completed"] or not stopped or not stopped["completed"] or not zero_ready:
        reasons.append("DATA_INCOMPLETE")
    if capture.truth_rows and min(row["z"] for row in capture.truth_rows) < 0.12:
        reasons.append("FALL_DETECTED")
    timing.refresh()
    if any(row.get("policy_wait_exit_reason") == "SIM_TIME_RESET" for row in timing.rows):
        reasons.append("SIM_TIME_RESET")
    if timing.latest_int("fsm_state") == FSM_PASSIVE:
        reasons.append("FSM_TRANSITION_FAILED")

    trial_result = g2_metrics.classify_trial_status(sorted(set(reasons)))
    metrics: Dict[str, object] = {}
    if active and stopped and capture.truth_rows:
        try:
            metrics = g2_metrics.compute_truth_metrics(
                capture.truth_rows,
                args.command_vx,
                active["sim_start"],
                active["sim_end"],
                stopped["sim_start"],
                stopped["sim_end"],
            )
        except ValueError as error:
            reasons.append("DATA_INCOMPLETE")
            trial_result = "INVALID"
            metrics["metrics_error"] = str(error)

    status = {
        "schema_version": 1,
        "trial_id": args.trial_id,
        "command_vx": args.command_vx,
        "trial_result": trial_result,
        "invalid_reasons": sorted(set(reasons)),
        "active_window": active,
        "stop_window": stopped,
        "truth_samples": len(capture.truth_rows),
        "foot_sequences": capture.foot_sequence,
        "timing_rows": len(timing.rows),
        **metrics,
    }
    if trial_result == "PASS":
        events.add("TRIAL_COMPLETED")
    else:
        events.add("TRIAL_INVALID", ",".join(status["invalid_reasons"]))
    capture.write_csvs(args.output_dir)
    events.write(os.path.join(args.output_dir, "events.csv"))
    with open(os.path.join(args.output_dir, "trial_status.json"), "w") as handle:
        json.dump(status, handle, indent=2, sort_keys=True)
    with open(os.path.join(args.output_dir, "trial_metrics.json"), "w") as handle:
        json.dump(status, handle, indent=2, sort_keys=True)
    print(json.dumps(status, indent=2, sort_keys=True))
    return 0 if trial_result != "INVALID" else 3


if __name__ == "__main__":
    raise SystemExit(main())
