#!/usr/bin/env python3
"""Capture one earth-world FixedStand/RL motion trial from a running simulator."""

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
from std_msgs.msg import Int8

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import earth_rl_metrics as metrics  # noqa: E402


FSM_FIXEDSTAND = 2
FSM_RL = 6


class TimingTail:
    def __init__(self, path: str):
        self.path = path
        self.rows: List[Dict[str, str]] = []
        self.latest: Optional[Dict[str, str]] = None

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
        try:
            return int(float((self.latest or {}).get(key, default)))
        except (TypeError, ValueError):
            return default


class PoseCapture:
    def __init__(self, model_name: str):
        self.model_name = model_name
        self.rows: List[Dict[str, float]] = []
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_models, queue_size=50)

    def _on_models(self, message: ModelStates) -> None:
        if self.model_name not in message.name:
            return
        index = message.name.index(self.model_name)
        pose = message.pose[index]
        twist = message.twist[index]
        roll, pitch, yaw = metrics.quaternion_to_euler(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        body_vx, body_vy = metrics.world_to_body(twist.linear.x, twist.linear.y, yaw)
        self.rows.append(
            {
                "sim_time": rospy.Time.now().to_sec(),
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "world_vx": twist.linear.x,
                "world_vy": twist.linear.y,
                "world_vz": twist.linear.z,
                "body_vx": body_vx,
                "body_vy": body_vy,
                "body_wz": twist.angular.z,
            }
        )


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


def make_twist(vx: float, vy: float, yaw_rate: float) -> Twist:
    message = Twist()
    message.linear.x = vx
    message.linear.y = vy
    message.angular.z = yaw_rate
    return message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--state", choices=("fixedstand", "rl"), required=True)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--stop-duration", type=float, default=0.0)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--timing-csv", required=True)
    parser.add_argument("--model-name", default="a1_gazebo")
    parser.add_argument("--wall-timeout", type=float, default=240.0)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("earth_rl_capture_%s" % args.trial_id.replace("-", "_"), anonymous=True)
    capture = PoseCapture(args.model_name)
    timing = TimingTail(args.timing_csv)
    state_pub = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=4)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    reasons: List[str] = []

    if not wait_until(lambda: capture.rows and rospy.Time.now().to_sec() > 0.0, 60.0):
        reasons.append("NO_GAZEBO_POSE")

    target_state = FSM_FIXEDSTAND if args.state == "fixedstand" else FSM_RL
    if not publish_state_until(state_pub, timing, FSM_FIXEDSTAND, 40.0):
        reasons.append("FIXEDSTAND_NOT_ENTERED")
    elif args.state == "rl" and not publish_state_until(state_pub, timing, FSM_RL, 40.0):
        reasons.append("RL_NOT_ENTERED")

    start_index = len(capture.rows)
    command = make_twist(args.vx, args.vy, args.yaw_rate)
    if args.state == "fixedstand":
        command = make_twist(0.0, 0.0, 0.0)
    active_ok = wait_sim_duration(args.duration, args.wall_timeout, cmd_pub, command)
    stop_ok = True
    stop_start = rospy.Time.now().to_sec()
    if args.stop_duration > 0.0:
        stop_ok = wait_sim_duration(args.stop_duration, args.wall_timeout, cmd_pub, make_twist(0.0, 0.0, 0.0))
    for _ in range(10):
        cmd_pub.publish(make_twist(0.0, 0.0, 0.0))
        time.sleep(0.02)

    trial_rows = capture.rows[start_index:]
    summary = metrics.summarize_motion(trial_rows)
    if not active_ok or not stop_ok:
        reasons.append("SIM_DURATION_INCOMPLETE")
    if timing.latest_int("fsm_state") not in (target_state, FSM_FIXEDSTAND):
        reasons.append("FSM_UNEXPECTED_FINAL_STATE")
    verdict = metrics.classify(summary, target_state, args.vx)
    if reasons and verdict == "PASS":
        verdict = "INVALID"

    timing.refresh()
    result = {
        "schema_version": 1,
        "trial_id": args.trial_id,
        "requested_state": args.state,
        "requested_state_id": target_state,
        "requested_vx": args.vx,
        "requested_vy": args.vy,
        "requested_yaw_rate": args.yaw_rate,
        "requested_duration": args.duration,
        "stop_duration": args.stop_duration,
        "stop_start_sim_time": stop_start,
        "final_fsm_state": timing.latest_int("fsm_state"),
        "timing_rows": len(timing.rows),
        "invalid_reasons": sorted(set(reasons)),
        "verdict": verdict,
        **summary,
    }
    write_csv(os.path.join(args.output_dir, "pose_samples.csv"), trial_rows)
    with open(os.path.join(args.output_dir, "trial_summary.json"), "w") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if verdict in ("PASS", "FAIL_FALL", "FAIL_ATTITUDE", "FAIL_NO_FORWARD_RESPONSE") else 3


if __name__ == "__main__":
    raise SystemExit(main())
