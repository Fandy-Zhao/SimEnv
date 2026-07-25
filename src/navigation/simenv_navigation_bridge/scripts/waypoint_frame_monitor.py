#!/usr/bin/env python3
"""DSV Waypoint Body-Frame Monitor.

Transforms every /navigation/way_point from map frame to robot body frame,
classifies it as FRONT / SIDE / REAR, and records all metrics to CSV.

Output columns:
  wall_time, sim_time,
  goal_x_map, goal_y_map, goal_z_map,
  robot_x_map, robot_y_map, robot_yaw_rad,
  goal_x_body, goal_y_body, goal_distance,
  goal_heading_body_rad, heading_error_rad,
  waypoint_region,
  planner_call_count, planner_success_count, planner_failure_count,
  goal_received_count, goal_rejected_count,
  waypoint_publish_count, last_waypoint_stamp,
  replan_count, replan_reason,
  rear_waypoint_ratio, consecutive_rear_waypoints,
  same_rear_waypoint_duration, rear_waypoint_goal_distance
"""

import csv
import math
import os
import time
from collections import namedtuple

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int8
from tf.transformations import euler_from_quaternion

PI = math.pi

Record = namedtuple("Record", [
    # Timestamps
    "wall_time", "sim_time",
    # Map-frame goal
    "goal_x_map", "goal_y_map", "goal_z_map",
    # Map-frame robot
    "robot_x_map", "robot_y_map", "robot_yaw_rad",
    # Body-frame goal
    "goal_x_body", "goal_y_body", "goal_distance",
    "goal_heading_body_rad", "heading_error_rad",
    # Classification
    "waypoint_region",
    # Planner counters
    "planner_call_count", "planner_success_count", "planner_failure_count",
    # Waypoint lifecycle
    "goal_received_count", "goal_rejected_count",
    "waypoint_publish_count", "last_waypoint_stamp",
    # Replan
    "replan_count", "replan_reason",
    # Rear-goal stats
    "rear_waypoint_ratio", "consecutive_rear_waypoints",
    "same_rear_waypoint_duration", "rear_waypoint_goal_distance",
])


def classify_waypoint_region(goal_x_body, heading_error_rad):
    """Classify waypoint as FRONT, SIDE, or REAR in robot body frame.

    FRONT: goal_x_body > 0 and |heading_error| <= 60 deg
    SIDE:  |heading_error| > 60 deg and |heading_error| <= 90 deg
    REAR:  goal_x_body < 0 or |heading_error| > 90 deg
    """
    heading_error_deg = abs(math.degrees(heading_error_rad))
    if goal_x_body > 0 and heading_error_deg <= 60.0:
        return "FRONT"
    elif heading_error_deg > 90.0 or goal_x_body < 0:
        return "REAR"
    else:
        return "SIDE"


class WaypointFrameMonitor:
    def __init__(self, output_path):
        self._output_path = output_path
        self._records = []

        # Latest robot state
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._sim_time = 0.0
        self._robot_valid = False

        # Latest waypoint (map frame)
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._goal_z = 0.0
        self._waypoint_valid = False
        self._last_waypoint_stamp = 0.0

        # Body-frame computed values
        self._goal_x_body = 0.0
        self._goal_y_body = 0.0
        self._goal_distance = 0.0
        self._goal_heading_body = 0.0
        self._heading_error = 0.0
        self._region = "NONE"

        # Waypoint lifecycle counters
        self._planner_call_count = 0
        self._planner_success_count = 0
        self._planner_failure_count = 0
        self._goal_received_count = 0
        self._goal_rejected_count = 0
        self._waypoint_publish_count = 0
        self._replan_count = 0
        self._replan_reason = ""

        # Rear-goal tracking
        self._region_history = []  # last N regions for ratio
        self._consecutive_rear = 0
        self._rear_start_time = 0.0
        self._last_rear_waypoint_pos = (0.0, 0.0)
        self._same_rear_duration = 0.0

        # Exploration state
        self._exploring = False
        self._enabled = False

        # Subscribers
        rospy.Subscriber("/Odometry", Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber("/navigation/way_point", PointStamped,
                         self._waypoint_cb, queue_size=5)
        rospy.Subscriber("/navigation/start_exploring", Bool,
                         self._exploring_cb, queue_size=1)
        rospy.Subscriber("/navigation/enabled", Bool,
                         self._enabled_cb, queue_size=1)

        # Periodic recording at 2 Hz (faster than the 1 Hz supervisor republish)
        self._record_timer = rospy.Timer(rospy.Duration(0.5), self._record_cb)

        # Write header
        with open(self._output_path, "w") as f:
            w = csv.writer(f)
            w.writerow(Record._fields)

        rospy.loginfo("WaypointFrameMonitor: recording to %s", self._output_path)

    # ── Callbacks ────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self._sim_time = msg.header.stamp.to_sec()
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._robot_yaw = yaw
        self._robot_valid = True

    def _waypoint_cb(self, msg):
        self._goal_x = msg.point.x
        self._goal_y = msg.point.y
        self._goal_z = msg.point.z
        self._last_waypoint_stamp = msg.header.stamp.to_sec()
        self._waypoint_valid = True
        self._waypoint_publish_count += 1

        # Transform to body frame
        self._compute_body_frame()

        # Classify region
        prev_region = self._region
        self._region = classify_waypoint_region(
            self._goal_x_body, self._heading_error)

        # Track region history (last 20)
        self._region_history.append(self._region)
        if len(self._region_history) > 20:
            self._region_history.pop(0)

        # Track consecutive rear waypoints
        if self._region == "REAR":
            if prev_region != "REAR":
                self._rear_start_time = time.time()
                self._last_rear_waypoint_pos = (self._goal_x, self._goal_y)
            self._consecutive_rear += 1
            # Check if same rear waypoint
            dx = self._goal_x - self._last_rear_waypoint_pos[0]
            dy = self._goal_y - self._last_rear_waypoint_pos[1]
            if math.sqrt(dx*dx + dy*dy) < 1e-3:
                self._same_rear_duration = time.time() - self._rear_start_time
        else:
            self._consecutive_rear = 0
            self._same_rear_duration = 0.0

    def _exploring_cb(self, msg):
        prev = self._exploring
        self._exploring = msg.data
        if self._exploring and not prev:
            self._planner_call_count += 1
        if not self._exploring and prev:
            self._replan_reason = "exploring_stopped"

    def _enabled_cb(self, msg):
        self._enabled = msg.data

    # ── Body-frame transform ─────────────────────────────────────────────

    def _compute_body_frame(self):
        """2D transform: map waypoint → robot body frame."""
        dx = self._goal_x - self._robot_x
        dy = self._goal_y - self._robot_y
        yaw = self._robot_yaw

        # Rotate into body frame
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        self._goal_x_body = dx * cos_yaw - dy * sin_yaw
        self._goal_y_body = dx * sin_yaw + dy * cos_yaw
        self._goal_distance = math.sqrt(dx*dx + dy*dy)
        self._goal_heading_body = math.atan2(self._goal_y_body, self._goal_x_body)
        self._heading_error = self._goal_heading_body

    # ── Periodic recording ───────────────────────────────────────────────

    def _record_cb(self, _event):
        rear_ratio = 0.0
        if self._region_history:
            rear_ratio = sum(1 for r in self._region_history if r == "REAR") / float(len(self._region_history))

        rear_goal_dist = 0.0
        if self._region == "REAR":
            rear_goal_dist = self._goal_distance

        r = Record(
            wall_time=time.time(),
            sim_time=self._sim_time,
            goal_x_map=self._goal_x,
            goal_y_map=self._goal_y,
            goal_z_map=self._goal_z,
            robot_x_map=self._robot_x,
            robot_y_map=self._robot_y,
            robot_yaw_rad=self._robot_yaw,
            goal_x_body=self._goal_x_body,
            goal_y_body=self._goal_y_body,
            goal_distance=self._goal_distance,
            goal_heading_body_rad=self._goal_heading_body,
            heading_error_rad=self._heading_error,
            waypoint_region=self._region,
            planner_call_count=self._planner_call_count,
            planner_success_count=self._planner_success_count,
            planner_failure_count=self._planner_failure_count,
            goal_received_count=self._goal_received_count,
            goal_rejected_count=self._goal_rejected_count,
            waypoint_publish_count=self._waypoint_publish_count,
            last_waypoint_stamp=self._last_waypoint_stamp,
            replan_count=self._replan_count,
            replan_reason=self._replan_reason,
            rear_waypoint_ratio=rear_ratio,
            consecutive_rear_waypoints=self._consecutive_rear,
            same_rear_waypoint_duration=self._same_rear_duration,
            rear_waypoint_goal_distance=rear_goal_dist,
        )
        self._records.append(r)

        with open(self._output_path, "a") as f:
            w = csv.writer(f)
            w.writerow(list(r))


if __name__ == "__main__":
    rospy.init_node("waypoint_frame_monitor")
    out = os.environ.get(
        "WAYPOINT_FRAME_CSV",
        os.path.join(os.path.dirname(__file__),
                     "../../../experiments/runs/0724_falco_dsv_single_floor_0p8",
                     "waypoint_body_frame.csv"))
    # Resolve to absolute
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    monitor = WaypointFrameMonitor(out)
    rospy.spin()
