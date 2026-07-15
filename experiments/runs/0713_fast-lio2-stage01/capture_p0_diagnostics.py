#!/usr/bin/env python3
"""Capture synchronized P0 truth, IMU, and FAST-LIO2 diagnostics."""

import json
import math
import os

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


DURATION_SECONDS = float(os.environ.get("FAST_LIO_P0_DIAGNOSTIC_SECONDS", "10"))
RUN_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT = os.path.join(RUN_DIR, "p0_diagnostic_metrics.json")


def norm3(vector):
    return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)


def angle_between(a, b):
    dot = abs(sum(left * right for left, right in zip(a, b)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def pose(message):
    p = message.pose.pose
    return [p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]


def pose_delta(rows):
    start, end = rows[0], rows[-1]
    position = math.dist(start[1:4], end[1:4])
    orientation = angle_between(start[4:], end[4:])
    return {
        "samples": len(rows),
        "simulation_duration_s": end[0] - start[0],
        "position_change_m": position,
        "orientation_change_deg": orientation,
    }


def summary(values):
    return {
        "samples": len(values),
        "mean": sum(values) / len(values),
        "max": max(values),
    } if values else None


class Capture:
    def __init__(self):
        self.start_time = None
        self.odom = []
        self.truth = []
        self.truth_linear = []
        self.truth_angular = []
        self.imu_gyro = []
        self.imu_acceleration = []
        self.complete = False
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback, queue_size=200)
        rospy.Subscriber("/ground_truth/base_w", Odometry, self.truth_callback, queue_size=1000)
        rospy.Subscriber("/livox/imu", Imu, self.imu_callback, queue_size=1000)

    def odom_callback(self, message):
        stamp = message.header.stamp.to_sec()
        if self.start_time is None:
            self.start_time = stamp
            rospy.loginfo("P0 diagnostic starts at %.3f s", stamp)
        if self.complete:
            return
        self.odom.append([stamp] + pose(message))
        if stamp - self.start_time >= DURATION_SECONDS:
            self.finish()

    def truth_callback(self, message):
        if self.start_time is None or self.complete:
            return
        self.truth.append([message.header.stamp.to_sec()] + pose(message))
        self.truth_linear.append(norm3(message.twist.twist.linear))
        self.truth_angular.append(norm3(message.twist.twist.angular))

    def imu_callback(self, message):
        if self.start_time is None or self.complete:
            return
        self.imu_gyro.append(norm3(message.angular_velocity))
        self.imu_acceleration.append(norm3(message.linear_acceleration))

    def finish(self):
        if self.complete:
            return
        self.complete = True
        report = {
            "test": "P0 localization-drift diagnostic",
            "target_simulation_duration_s": DURATION_SECONDS,
            "odometry": pose_delta(self.odom),
            "ground_truth": pose_delta(self.truth),
            "ground_truth_linear_speed_mps": summary(self.truth_linear),
            "ground_truth_angular_speed_radps": summary(self.truth_angular),
            "imu_angular_speed_radps": summary(self.imu_gyro),
            "imu_acceleration_norm_mps2": summary(self.imu_acceleration),
        }
        with open(OUTPUT, "w") as stream:
            json.dump(report, stream, indent=2)
            stream.write("\n")
        rospy.loginfo("P0 diagnostic completed: %s", OUTPUT)
        rospy.signal_shutdown("capture complete")


if __name__ == "__main__":
    if not 1.0 <= DURATION_SECONDS <= 60.0:
        raise ValueError("FAST_LIO_P0_DIAGNOSTIC_SECONDS must be between 1 and 60")
    rospy.init_node("fast_lio_p0_diagnostic")
    Capture()
    rospy.spin()
