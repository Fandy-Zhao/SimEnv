#!/usr/bin/env python3
"""Run and record a truth-distance-bounded FAST-LIO2 straight-line test.

Start junior_ctrl, enter its RL state (keyboard ``6``), then run this script.
It publishes a low-speed /cmd_vel until the Gazebo ground-truth forward
projection reaches the requested distance.  It is intentionally an experiment
harness, not a navigation component.
"""

import csv
import json
import math
import os
import re
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


RUN_DIR = os.path.dirname(os.path.abspath(__file__))


def env_float(name, default, lower, upper):
    try:
        value = float(os.environ.get(name, default))
    except ValueError as error:
        raise ValueError(f"{name} must be numeric") from error
    if not lower <= value <= upper:
        raise ValueError(f"{name} must be between {lower} and {upper}")
    return value


TARGET_M = env_float("FAST_LIO_P1_TARGET_M", "1.0", 0.2, 5.0)
SPEED_MPS = env_float("FAST_LIO_P1_SPEED_MPS", "0.15", 0.05, 0.30)
SETTLE_S = env_float("FAST_LIO_P1_SETTLE_S", "2.0", 0.0, 10.0)
TIMEOUT_S = env_float("FAST_LIO_P1_TIMEOUT_S", "90.0", 10.0, 300.0)
TAG = os.environ.get("FAST_LIO_P1_RUN_TAG", f"p1_straight_{TARGET_M:g}m").replace(".", "p")
if not re.fullmatch(r"[A-Za-z0-9_-]+", TAG):
    raise ValueError("FAST_LIO_P1_RUN_TAG may contain only letters, digits, '_' and '-'")


def yaw_from_row(row):
    x, y, z, w = row[4:8]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_row(message):
    pose = message.pose.pose
    return [
        message.header.stamp.to_sec(),
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]


class StraightTest:
    def __init__(self):
        self.truth = None
        self.odom = None
        self.truth_rows = []
        self.odom_rows = []
        self.truth_file = open(os.path.join(RUN_DIR, f"ground_truth_{TAG}.csv"), "w", newline="")
        self.odom_file = open(os.path.join(RUN_DIR, f"odometry_{TAG}.csv"), "w", newline="")
        header = ["sim_time_s", "x", "y", "z", "qx", "qy", "qz", "qw"]
        self.truth_writer = csv.writer(self.truth_file)
        self.odom_writer = csv.writer(self.odom_file)
        self.truth_writer.writerow(header)
        self.odom_writer.writerow(header)
        self.cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/ground_truth/base_w", Odometry, self.truth_callback, queue_size=1000)
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback, queue_size=200)

    def truth_callback(self, message):
        self.truth = pose_row(message)

    def odom_callback(self, message):
        self.odom = pose_row(message)

    def publish(self, linear_x=0.0):
        message = Twist()
        message.linear.x = linear_x
        self.cmd.publish(message)

    def wait_for_inputs(self):
        deadline = time.monotonic() + 30.0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and (self.truth is None or self.odom is None):
            if time.monotonic() > deadline:
                raise RuntimeError("Timed out waiting for /ground_truth/base_w and /Odometry")
            self.publish(0.0)
            rate.sleep()

    @staticmethod
    def distance_metrics(rows, initial_yaw):
        start, end = rows[0], rows[-1]
        dx, dy = end[1] - start[1], end[2] - start[2]
        forward = dx * math.cos(initial_yaw) + dy * math.sin(initial_yaw)
        lateral = -dx * math.sin(initial_yaw) + dy * math.cos(initial_yaw)
        return {
            "samples": len(rows),
            "simulation_duration_s": end[0] - start[0],
            "forward_displacement_m": forward,
            "lateral_displacement_m": lateral,
            "planar_displacement_m": math.hypot(dx, dy),
            "z_change_m": end[3] - start[3],
            "start_pose": start[1:],
            "end_pose": end[1:],
            "all_values_finite": all(math.isfinite(v) for row in rows for v in row[1:]),
        }

    def run(self):
        self.wait_for_inputs()
        settle_start = rospy.Time.now().to_sec()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - settle_start < SETTLE_S:
            self.publish(0.0)
            rate.sleep()

        initial_truth = self.truth
        initial_yaw = yaw_from_row(initial_truth)
        start_sim = rospy.Time.now().to_sec()
        wall_deadline = time.monotonic() + TIMEOUT_S / 0.03
        completed = False
        while not rospy.is_shutdown():
            current = self.truth
            dx, dy = current[1] - initial_truth[1], current[2] - initial_truth[2]
            forward = dx * math.cos(initial_yaw) + dy * math.sin(initial_yaw)
            self.truth_rows.append(current)
            self.truth_writer.writerow(current)
            if self.odom is not None:
                self.odom_rows.append(self.odom)
                self.odom_writer.writerow(self.odom)
            if forward >= TARGET_M:
                completed = True
                break
            if rospy.Time.now().to_sec() - start_sim >= TIMEOUT_S or time.monotonic() >= wall_deadline:
                break
            self.publish(SPEED_MPS)
            rate.sleep()

        stop_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - stop_start < SETTLE_S:
            self.publish(0.0)
            if self.truth is not None:
                self.truth_rows.append(self.truth)
                self.truth_writer.writerow(self.truth)
            if self.odom is not None:
                self.odom_rows.append(self.odom)
                self.odom_writer.writerow(self.odom)
            rate.sleep()
        for _ in range(10):
            self.publish(0.0)

        self.truth_file.close()
        self.odom_file.close()
        report = {
            "test": "P1 reduced straight-line RL-policy test",
            "run_tag": TAG,
            "target_forward_distance_m": TARGET_M,
            "command_linear_x_mps": SPEED_MPS,
            "completed_target": completed,
            "ground_truth": self.distance_metrics(self.truth_rows, initial_yaw) if len(self.truth_rows) >= 2 else None,
            "odometry": self.distance_metrics(self.odom_rows, initial_yaw) if len(self.odom_rows) >= 2 else None,
        }
        with open(os.path.join(RUN_DIR, f"{TAG}_metrics.json"), "w") as stream:
            json.dump(report, stream, indent=2)
            stream.write("\n")
        rospy.loginfo("P1 straight test complete: %s", report)


def main():
    rospy.init_node("fast_lio_p1_straight")
    test = StraightTest()
    try:
        test.run()
    finally:
        for _ in range(10):
            test.publish(0.0)


if __name__ == "__main__":
    main()
