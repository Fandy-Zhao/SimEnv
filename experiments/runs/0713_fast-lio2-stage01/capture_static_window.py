#!/usr/bin/env python3
"""Capture a fixed ROS-simulation-time FAST-LIO2 static-test window."""

import csv
import json
import math
import os
import re

import rospy
from nav_msgs.msg import Odometry


RUN_DIR = os.path.dirname(os.path.abspath(__file__))


def read_duration():
    value = os.environ.get("FAST_LIO_P0_DURATION_SECONDS", "60")
    try:
        duration = float(value)
    except ValueError as error:
        raise ValueError("FAST_LIO_P0_DURATION_SECONDS must be numeric") from error
    if not 1.0 <= duration <= 60.0:
        raise ValueError("FAST_LIO_P0_DURATION_SECONDS must be between 1 and 60")
    return duration


DURATION_SECONDS = read_duration()
DEFAULT_TAG = f"p0_stand_sim{DURATION_SECONDS:g}".replace(".", "p")
RUN_TAG = os.environ.get("FAST_LIO_P0_RUN_TAG", DEFAULT_TAG)
if not re.fullmatch(r"[A-Za-z0-9_-]+", RUN_TAG):
    raise ValueError("FAST_LIO_P0_RUN_TAG may contain only letters, digits, '_' and '-'")

ODOM_CSV = os.path.join(RUN_DIR, f"odometry_{RUN_TAG}.csv")
TRUTH_CSV = os.path.join(RUN_DIR, f"ground_truth_{RUN_TAG}.csv")
METRICS_JSON = os.path.join(RUN_DIR, f"{RUN_TAG}_metrics.json")


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


def pose_metrics(rows):
    start, end = rows[0], rows[-1]
    distance = math.dist(start[1:4], end[1:4])
    dot = abs(sum(a * b for a, b in zip(start[4:8], end[4:8])))
    angle = 2.0 * math.degrees(math.acos(max(-1.0, min(1.0, dot))))
    finite = all(math.isfinite(value) for row in rows for value in row[1:])
    return {
        "samples": len(rows),
        "simulation_duration_s": end[0] - start[0],
        "position_change_m": distance,
        "orientation_change_deg": angle,
        "all_values_finite": finite,
        "start_pose": start[1:],
        "end_pose": end[1:],
    }


class Capture:
    def __init__(self):
        self.start_time = None
        self.odom_rows = []
        self.truth_rows = []
        self.done = False

        self.odom_file = open(ODOM_CSV, "w", newline="")
        self.truth_file = open(TRUTH_CSV, "w", newline="")
        self.odom_writer = csv.writer(self.odom_file)
        self.truth_writer = csv.writer(self.truth_file)
        header = ["sim_time_s", "x", "y", "z", "qx", "qy", "qz", "qw"]
        self.odom_writer.writerow(header)
        self.truth_writer.writerow(header)

        rospy.Subscriber("/Odometry", Odometry, self.odom_callback, queue_size=200)
        rospy.Subscriber("/ground_truth/base_w", Odometry, self.truth_callback, queue_size=1000)

    def odom_callback(self, message):
        row = pose_row(message)
        if self.start_time is None:
            self.start_time = row[0]
            rospy.loginfo("P0 static capture starts at %.3f s", self.start_time)
        self.odom_rows.append(row)
        self.odom_writer.writerow(row)
        if row[0] - self.start_time >= DURATION_SECONDS:
            self.finish()

    def truth_callback(self, message):
        if self.start_time is None or self.done:
            return
        row = pose_row(message)
        self.truth_rows.append(row)
        self.truth_writer.writerow(row)

    def finish(self):
        if self.done:
            return
        self.done = True
        self.odom_file.close()
        self.truth_file.close()
        metrics = {
            "test": "P0 stationary, fixed stand",
            "run_tag": RUN_TAG,
            "target_simulation_duration_s": DURATION_SECONDS,
            "odometry": pose_metrics(self.odom_rows),
            "ground_truth": pose_metrics(self.truth_rows) if len(self.truth_rows) >= 2 else None,
        }
        with open(METRICS_JSON, "w") as output:
            json.dump(metrics, output, indent=2)
            output.write("\n")
        rospy.loginfo("P0 static capture completed: %s", METRICS_JSON)
        rospy.signal_shutdown("capture complete")


def main():
    rospy.init_node("fast_lio_p0_static_capture")
    capture = Capture()
    rospy.spin()
    if not capture.done:
        capture.odom_file.close()
        capture.truth_file.close()


if __name__ == "__main__":
    main()
