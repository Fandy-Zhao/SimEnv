#!/usr/bin/env python3
"""
Exploration Result Recorder — records map, route, goals, timing, and health
data during a single-floor exploration run.

Usage:
  rosrun simenv_navigation_bridge exploration_result_recorder.py \
    _output_dir:=/path/to/output

Design principles:
  - Low coupling: subscribes to existing topics; does NOT modify DSV/FALCO/FAST-LIO2.
  - ROS-sim-time primary: all timestamps logged in both sim time and wall time.
  - Completion detection via DSV native signal + composite fallback.
  - Saves partial results on any failure; never corrupts existing data.
"""

import argparse
import csv
import json
import math
import os
import signal
import sys
import threading
import time
import traceback
from collections import deque
from datetime import datetime, timezone

import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped with tf2
import tf2_ros
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Int8

from simenv_navigation_bridge.exploration_metrics import (
    DEFAULT_ROUTE_MAX_SPEED_MPS,
    DEFAULT_ROUTE_MAX_STEP_M,
    DEFAULT_TARGET_FRAME,
    REJECT_SPEED_EXCEEDED,
    REJECT_STEP_EXCEEDED,
    RouteAccumulator,
    RoutePolicy,
    compute_route_length,
)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
GOAL_STATUS_PUBLISHED = "PUBLISHED"
GOAL_STATUS_REACHED = "REACHED"
GOAL_STATUS_REPLACED = "REPLACED"
GOAL_STATUS_REJECTED = "REJECTED"
GOAL_STATUS_INVALID = "INVALID"
GOAL_STATUS_UNREACHED_AT_FINISH = "UNREACHED_AT_FINISH"

COMPLETION_METHOD_DSV_NATIVE = "dsv_native_stop_signal"
COMPLETION_METHOD_COMPOSITE = "composite_quiet_window"
COMPLETION_METHOD_TIMEOUT = "exploration_timeout"
COMPLETION_METHOD_MANUAL = "manual_stop"
COMPLETION_METHOD_MINIMAL_MAP = "minimal_map_validation"

VERDICT_COMPLETE = "COMPLETE"
VERDICT_TIMEOUT = "EXPLORATION_TIMEOUT"
VERDICT_FAILURE = "EXPLORATION_FAILURE"
VERDICT_PARTIAL = "PARTIAL_RESULTS_SAVED"
VERDICT_MINIMAL_MAP = "MINIMAL_MAP_VALIDATION"


# ---------------------------------------------------------------------------
# Helper: timestamp helpers
# ---------------------------------------------------------------------------
def wall_now():
    return datetime.now(timezone.utc)


def wall_iso(ts=None):
    return (ts if ts else wall_now()).isoformat()


def wall_elapsed(start_wall):
    return (wall_now() - start_wall).total_seconds()


# ---------------------------------------------------------------------------
# Exploration Result Recorder
# ---------------------------------------------------------------------------
class ExplorationResultRecorder:
    def __init__(self):
        # ── Configuration parameters ──
        self.output_dir = rospy.get_param("~output_dir", "")
        self.run_id = rospy.get_param("~run_id", "unknown")
        self.max_sim_time = float(rospy.get_param("~max_sim_time", 1800.0))
        self.finish_quiet_time = float(rospy.get_param("~finish_quiet_time", 60.0))
        self.map_stable_wait = float(rospy.get_param("~map_stable_wait", 5.0))
        self.goal_dedup_distance = float(rospy.get_param("~goal_dedup_distance", 0.5))
        self.map_growth_threshold = float(rospy.get_param("~map_growth_threshold", 0.1))
        self.robot_motion_threshold = float(rospy.get_param("~robot_motion_threshold", 0.2))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 10.0))

        # ── Minimal map validation mode (diagnostic only, not for production) ──
        self.minimal_map_validation = rospy.get_param("~minimal_map_validation", False)
        self.stop_after_map_updates = int(rospy.get_param("~stop_after_map_updates", 3))

        # Topic names (configurable via ROS params)
        self.clock_topic = rospy.get_param("~clock_topic", "/clock")
        legacy_odometry_topic = str(
            rospy.get_param("~odometry_topic", "")).strip()
        configured_trajectory_topic = str(
            rospy.get_param("~trajectory_pose_topic", "")).strip()
        self.trajectory_pose_topic = (
            configured_trajectory_topic
            or legacy_odometry_topic
            or "/navigation/state_estimation"
        )
        # Backward-compatible alias for existing reports and direct callers.
        self.odometry_topic = self.trajectory_pose_topic
        if legacy_odometry_topic and not configured_trajectory_topic:
            rospy.logwarn(
                "[Recorder] ~odometry_topic is deprecated; use "
                "~trajectory_pose_topic instead")
        self.trajectory_target_frame = str(rospy.get_param(
            "~trajectory_target_frame", DEFAULT_TARGET_FRAME)).strip()
        self.route_max_speed_mps = float(rospy.get_param(
            "~route_max_speed_mps", DEFAULT_ROUTE_MAX_SPEED_MPS))
        self.route_max_step_m = float(rospy.get_param(
            "~route_max_step_m", DEFAULT_ROUTE_MAX_STEP_M))
        self.transform_timeout = float(rospy.get_param(
            "~transform_timeout", 0.05))
        self._route_policy = RoutePolicy(
            target_frame=self.trajectory_target_frame,
            max_speed_mps=self.route_max_speed_mps,
            max_step_m=self.route_max_step_m,
        )
        self.goal_topic = rospy.get_param("~goal_topic", "/navigation/dsv/next_goal")
        self.map_topic = rospy.get_param("~map_topic", "/navigation/dsv/occupancy_grid_map")
        self.octomap_binary_topic = rospy.get_param("~octomap_binary_topic",
                                                      "/navigation/dsvplanner/octomap_binary")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.stop_signal_topic = rospy.get_param("~stop_signal_topic",
                                                   "/navigation/stop_exploring")
        self.start_signal_topic = rospy.get_param("~start_signal_topic",
                                                    "/navigation/start_exploring")
        self.enabled_topic = rospy.get_param("~enabled_topic", "/navigation/enabled")
        self.fsm_state_topic = rospy.get_param("~fsm_state_topic", "/fsm/state_cmd")
        self.path_topic = rospy.get_param("~path_topic", "/navigation/path")
        self.frontier_topic = rospy.get_param("~frontier_topic",
                                                "/navigation/local_frontier")
        self.terrain_topic = rospy.get_param("~terrain_topic", "/navigation/terrain_map")
        self.cloud_registered_topic = rospy.get_param("~cloud_registered_topic",
                                                        "/cloud_registered")

        # ── Internal state ──
        self._lock = threading.RLock()
        self._start_wall = wall_now()
        self._process_start_wall = self._start_wall

        # Simulation time
        self._current_sim_time = rospy.Time(0)
        self._sim_time_started = False
        self._clock_reset_detected = False
        self._clock_jump_detected = False
        self._clock_sample_count = 0
        self._last_clock = rospy.Time(0)
        self._sim_clock_history = deque(maxlen=20)

        # Exploration timing
        self._exploration_start_sim_time = None
        self._exploration_end_sim_time = None
        self._exploration_started = False
        self._exploration_completed = False

        # Odometry
        self._odom_samples = []
        self._odom_count = 0
        self._odom_healthy = False
        self._last_odom_sim_time = rospy.Time(0)
        self._last_odom_wall_time = wall_now()
        self._odom_nan_detected = False
        self._odom_jump_detected = False
        self._last_position = None
        self._route_accumulator = RouteAccumulator(self._route_policy)
        self._final_route_metrics = None
        self._trajectory_transform_drop_count = 0
        self._trajectory_source_frames = set()
        self._robot_fall_detected = False
        self._robot_height_history = deque(maxlen=50)

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Goals
        self._raw_goal_count = 0
        self._goals = []           # all received goals
        self._unique_goals = []    # deduplicated logical goals
        self._reached_goals = 0
        self._unreached_goals = 0
        self._last_goal = None
        self._last_unique_goal_sim_time = rospy.Time(0)
        self._no_new_goal_window_start = None

        # Map
        self._last_map_msg = None
        self._map_count = 0
        self._map_healthy = False
        self._last_map_sim_time = rospy.Time(0)
        self._map_known_area_series = deque(maxlen=100)

        # OctoMap (3D)
        self._octomap_binary_msg = None
        self._octomap_node_count_series = deque(maxlen=100)

        # Health
        self._noeff_count = 0
        self._cmd_vel_count = 0
        self._last_cmd_vel_time = wall_now()
        self._planner_alive = False
        self._last_planner_activity = wall_now()
        self._falco_alive = False
        self._last_falco_activity = wall_now()
        self._bridge_alive = False
        self._last_bridge_activity = wall_now()
        self._fsm_state = 0
        self._trotting_achieved = False
        self._navigation_enabled = False

        # Completion
        self._completion_method = None
        self._completion_reason = ""
        self._completion_sim_time = None
        self._dvs_native_stop_received = False
        self._dvs_native_stop_sim_time = None

        # Frontier / planner status
        self._last_frontier_count = 0
        self._frontier_history = deque(maxlen=50)
        self._last_dsv_runtime = 0.0

        # Health events log
        self._health_events = []

        # Shutdown
        self._shutdown_requested = False

        # ── Validate output directory ──
        if not self.output_dir:
            rospy.logerr("[Recorder] No output_dir specified. Recording disabled.")
            rospy.signal_shutdown("no output_dir")
            return

        if os.path.exists(self.output_dir):
            existing = os.listdir(self.output_dir)
            if existing:
                rospy.logwarn("[Recorder] Output dir exists with %d entries: %s",
                              len(existing), self.output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        # ── Create subdirectories ──
        for sub in ["config", "map", "route", "goals", "plots", "timing", "logs"]:
            os.makedirs(os.path.join(self.output_dir, sub), exist_ok=True)

        # ── Subscribers ──
        self._subs = []
        self._subs.append(rospy.Subscriber(self.clock_topic, rospy.AnyMsg,
                                            self._clock_cb, queue_size=10))
        self._subs.append(rospy.Subscriber(self.trajectory_pose_topic, Odometry,
                                            self._odom_cb, queue_size=10))
        self._subs.append(rospy.Subscriber(self.goal_topic, PointStamped,
                                            self._goal_cb, queue_size=20))
        self._subs.append(rospy.Subscriber(self.stop_signal_topic, Bool,
                                            self._stop_cb, queue_size=1))
        self._subs.append(rospy.Subscriber(self.start_signal_topic, Bool,
                                            self._start_signal_cb, queue_size=1))
        self._subs.append(rospy.Subscriber(self.cmd_vel_topic, Twist,
                                            self._cmd_vel_cb, queue_size=5))
        self._subs.append(rospy.Subscriber(self.fsm_state_topic, Int8,
                                            self._fsm_state_cb, queue_size=5))
        self._subs.append(rospy.Subscriber(self.enabled_topic, Bool,
                                            self._enabled_cb, queue_size=5))
        self._subs.append(rospy.Subscriber(self.path_topic, Path,
                                            self._path_cb, queue_size=5))
        self._subs.append(rospy.Subscriber(self.cloud_registered_topic, PointCloud2,
                                            self._cloud_cb, queue_size=3))

        # Map topic subscriber — try OccupancyGrid first, then PointCloud2
        self._map_is_occupancy_grid = False
        self._subs.append(rospy.Subscriber(self.map_topic, rospy.AnyMsg,
                                            self._map_any_cb, queue_size=3))

        # OctoMap binary subscriber (for 2D projection)
        self._subs.append(rospy.Subscriber(self.octomap_binary_topic, rospy.AnyMsg,
                                            self._octomap_cb, queue_size=3))

        # Frontier subscriber
        self._subs.append(rospy.Subscriber(self.frontier_topic, PointCloud2,
                                            self._frontier_cb, queue_size=3))

        # DSV runtime (planner activity heartbeat)
        try:
            self._subs.append(rospy.Subscriber("/navigation/dsv/runtime", Float32,
                                                self._dsv_runtime_cb, queue_size=5))
        except Exception:
            pass

        # ── Publishers ──
        self._stop_repub = rospy.Publisher("/navigation/stop_exploring", Bool,
                                            queue_size=1, latch=True)
        self._zero_vel_pub = None

        # ── Timer for health monitoring (1 Hz) ──
        self._health_timer = rospy.Timer(rospy.Duration(1.0), self._health_check)

        # ── Timer for completion detection (2 Hz) ──
        self._completion_timer = rospy.Timer(rospy.Duration(0.5),
                                              self._check_completion)

        # ── Timer for periodic status log (10 s) ──
        self._status_timer = rospy.Timer(rospy.Duration(10.0), self._status_log)

        # ── Shutdown handler ──
        rospy.on_shutdown(self._on_shutdown)

        # ── Save runtime config immediately ──
        self._save_runtime_config()

        rospy.loginfo("[Recorder] Initialized. Output: %s", self.output_dir)
        rospy.loginfo("[Recorder] Completion: dsv_native + composite "
                       "(quiet=%ss, max_sim=%ss)",
                       self.finish_quiet_time, self.max_sim_time)

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------
    def _clock_cb(self, msg):
        """Parse /clock from rosgraph_msgs/Clock."""
        try:
            # rosgraph_msgs/Clock has 'clock' field of type rosgraph_msgs/Time
            clock_msg = msg
            sim_sec = clock_msg.clock.secs
            sim_nsec = clock_msg.clock.nsecs
        except Exception:
            try:
                sim_sec = msg.secs
                sim_nsec = msg.nsecs
            except Exception:
                return

        sim_t = rospy.Time(sim_sec, sim_nsec)
        with self._lock:
            self._clock_sample_count += 1

            if not self._sim_time_started:
                self._sim_time_started = True
                self._last_clock = sim_t
                self._current_sim_time = sim_t
                return

            # Detect clock jumps (backward or large forward)
            if self._last_clock.to_sec() > 0:
                delta = sim_t.to_sec() - self._last_clock.to_sec()
                if delta < -0.5:
                    self._clock_reset_detected = True
                    self._health_events.append({
                        "wall_time": wall_iso(),
                        "sim_time": sim_t.to_sec(),
                        "event": "CLOCK_RESET",
                        "detail": f"Clock jumped backward: {delta:.3f}s"
                    })
                    rospy.logerr("[Recorder] CLOCK RESET detected: delta=%.3f", delta)
                elif delta > 30.0:
                    self._clock_jump_detected = True
                    self._health_events.append({
                        "wall_time": wall_iso(),
                        "sim_time": sim_t.to_sec(),
                        "event": "CLOCK_JUMP",
                        "detail": f"Large forward jump: {delta:.3f}s"
                    })
                    rospy.logwarn("[Recorder] Clock forward jump: delta=%.3f", delta)

            self._last_clock = sim_t
            self._current_sim_time = sim_t
            self._sim_clock_history.append(sim_t.to_sec())

    def _normalize_trajectory_pose(self, msg):
        """Return a target-frame PoseStamped, or ``None`` when TF is missing."""
        source_frame = str(msg.header.frame_id).strip()
        with self._lock:
            self._trajectory_source_frames.add(source_frame or "<empty>")
        if not source_frame:
            with self._lock:
                self._trajectory_transform_drop_count += 1
            rospy.logwarn_throttle(
                5.0, "[Recorder] Dropping trajectory pose with empty frame_id")
            return None, False, source_frame

        source_pose = PoseStamped()
        source_pose.header = msg.header
        source_pose.pose = msg.pose.pose
        if source_frame == self.trajectory_target_frame:
            source_pose.header.frame_id = self.trajectory_target_frame
            return source_pose, False, source_frame

        try:
            target_pose = self._tf_buffer.transform(
                source_pose,
                self.trajectory_target_frame,
                rospy.Duration(self.transform_timeout),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
            with self._lock:
                self._trajectory_transform_drop_count += 1
            rospy.logwarn_throttle(
                5.0,
                "[Recorder] Dropping trajectory pose: cannot transform %s -> %s: %s",
                source_frame, self.trajectory_target_frame, exc)
            return None, False, source_frame

        target_pose.header.frame_id = self.trajectory_target_frame
        return target_pose, True, source_frame

    def _odom_cb(self, msg):
        """Record one pose after enforcing the configured target frame."""
        with self._lock:
            if not self._sim_time_started:
                return

        normalized, transform_applied, source_frame = (
            self._normalize_trajectory_pose(msg))
        if normalized is None:
            return

        with self._lock:
            sim_t = msg.header.stamp
            w = wall_now()
            pos = normalized.pose.position
            orient = normalized.pose.orientation
            vel = msg.twist.twist.linear

            # NaN/Inf check
            for val in [pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w]:
                if not math.isfinite(val):
                    self._odom_nan_detected = True
                    self._health_events.append({
                        "wall_time": wall_iso(w),
                        "sim_time": sim_t.to_sec(),
                        "event": "ODOM_NAN",
                        "detail": "NaN/Inf in odometry position/orientation"
                    })
                    return

            # Track robot height for fall detection
            self._robot_height_history.append(pos.z)

            # Compute roll/pitch from quaternion
            roll, pitch, yaw = self._quat_to_rpy(orient)

            self._last_position = (pos.x, pos.y, pos.z)

            sample = {
                "index": self._odom_count,
                "sim_time": sim_t.to_sec(),
                "wall_time": wall_iso(w),
                "x": pos.x, "y": pos.y, "z": pos.z,
                "qx": orient.x, "qy": orient.y, "qz": orient.z, "qw": orient.w,
                "roll": roll, "pitch": pitch, "yaw": yaw,
                "linear_velocity_x": vel.x,
                "linear_velocity_y": vel.y,
                "linear_velocity_z": vel.z,
                "angular_velocity_x": msg.twist.twist.angular.x,
                "angular_velocity_y": msg.twist.twist.angular.y,
                "angular_velocity_z": msg.twist.twist.angular.z,
                "frame_id": self.trajectory_target_frame,
                "child_frame_id": msg.child_frame_id,
                "source_frame_id": source_frame,
                "target_frame_id": self.trajectory_target_frame,
                "source_topic": self.trajectory_pose_topic,
                "transform_applied": transform_applied,
            }
            evaluation = self._route_accumulator.add(sample)
            if (evaluation is not None and not evaluation.accepted
                    and evaluation.reject_reason in
                    (REJECT_SPEED_EXCEEDED, REJECT_STEP_EXCEEDED)):
                self._odom_jump_detected = True
                self._health_events.append({
                    "wall_time": wall_iso(w),
                    "sim_time": sim_t.to_sec(),
                    "event": "ODOM_ROUTE_SEGMENT_REJECTED",
                    "detail": (
                        f"reason={evaluation.reject_reason} "
                        f"distance={evaluation.distance_m:.3f}m "
                        f"dt={evaluation.dt:.3f}s "
                        f"speed={evaluation.speed_mps:.3f}m/s"
                    ),
                })
            self._odom_samples.append(sample)
            self._odom_count += 1
            self._last_odom_sim_time = sim_t
            self._last_odom_wall_time = w

    def _goal_cb(self, msg):
        """Record DSV next_goal."""
        with self._lock:
            sim_t = msg.header.stamp
            if sim_t.to_sec() == 0:
                sim_t = self._current_sim_time
            w = wall_now()
            pos = msg.point

            self._raw_goal_count += 1

            # NaN check
            if not all(math.isfinite(v) for v in [pos.x, pos.y, pos.z]):
                return

            goal_entry = {
                "goal_index": self._raw_goal_count,
                "received_sim_time": sim_t.to_sec(),
                "received_wall_time": wall_iso(w),
                "x": pos.x, "y": pos.y, "z": pos.z,
                "frame_id": msg.header.frame_id,
                "source_topic": self.goal_topic,
                "is_duplicate": False,
                "reached": False,
                "reached_sim_time": 0.0,
                "reached_distance_m": -1.0,
                "status": GOAL_STATUS_PUBLISHED,
            }

            # Deduplication
            is_dup = False
            if self._last_goal is not None:
                dx = pos.x - self._last_goal["x"]
                dy = pos.y - self._last_goal["y"]
                if math.sqrt(dx * dx + dy * dy) < self.goal_dedup_distance:
                    is_dup = True
                    goal_entry["is_duplicate"] = True

            self._goals.append(goal_entry)

            if not is_dup:
                goal_entry["goal_index_unique"] = len(self._unique_goals) + 1
                self._unique_goals.append(goal_entry)
                self._last_unique_goal_sim_time = sim_t
                self._no_new_goal_window_start = None  # reset quiet window
            else:
                # Update the last unique goal's status to REPLACED if this is newer
                if self._unique_goals:
                    last_u = self._unique_goals[-1]
                    if last_u["status"] == GOAL_STATUS_PUBLISHED:
                        last_u["status"] = GOAL_STATUS_REPLACED

            self._last_goal = goal_entry

    def _stop_cb(self, msg):
        """Detect DSV native stop signal."""
        if msg.data:
            with self._lock:
                self._dvs_native_stop_received = True
                self._dvs_native_stop_sim_time = self._current_sim_time
                self._health_events.append({
                    "wall_time": wall_iso(),
                    "sim_time": self._current_sim_time.to_sec(),
                    "event": "DSV_NATIVE_STOP",
                    "detail": "DSV published /navigation/stop_exploring=true"
                })
            rospy.loginfo("[Recorder] DSV native stop signal received at sim=%.1f",
                          self._current_sim_time.to_sec())

    def _start_signal_cb(self, msg):
        """Detect exploration start signal."""
        if msg.data and not self._exploration_started:
            with self._lock:
                self._exploration_started = True
                self._exploration_start_sim_time = self._current_sim_time
                self._exploration_start_wall = wall_now()
                self._health_events.append({
                    "wall_time": wall_iso(),
                    "sim_time": self._current_sim_time.to_sec(),
                    "event": "EXPLORATION_STARTED",
                    "detail": "/navigation/start_exploring=true"
                })
            rospy.loginfo("[Recorder] Exploration started at sim=%.1f wall=%s",
                          self._current_sim_time.to_sec(),
                          wall_iso(self._exploration_start_wall))

    def _cmd_vel_cb(self, msg):
        """Track cmd_vel for bridge liveness."""
        self._cmd_vel_count += 1
        self._last_cmd_vel_time = wall_now()
        self._bridge_alive = True
        self._last_bridge_activity = wall_now()
        # Check for NoEff (zero velocity despite being enabled)
        with self._lock:
            if (self._navigation_enabled and self._fsm_state == 4 and
                    abs(msg.linear.x) < 0.001 and abs(msg.angular.z) < 0.001):
                self._noeff_count += 1

    def _fsm_state_cb(self, msg):
        self._fsm_state = msg.data
        if msg.data == 4:
            self._trotting_achieved = True

    def _enabled_cb(self, msg):
        self._navigation_enabled = msg.data

    def _path_cb(self, msg):
        """Track FALCO paths."""
        self._falco_alive = True
        self._last_falco_activity = wall_now()

    def _cloud_cb(self, msg):
        """Track cloud_registered for liveness."""
        pass

    def _map_any_cb(self, msg):
        """Handle map topic — AnyMsg (deserialize to OccupancyGrid or PointCloud2)."""
        with self._lock:
            self._map_count += 1
        self._last_map_sim_time = self._current_sim_time

        try:
            # rospy.AnyMsg exposes _connection_header with the topic type
            conn_header = getattr(msg, '_connection_header', {})
            msg_type = conn_header.get('type', '')

            if 'OccupancyGrid' in msg_type:
                # Deserialize AnyMsg → nav_msgs/OccupancyGrid
                from nav_msgs.msg import OccupancyGrid
                try:
                    og = OccupancyGrid()
                    og.deserialize(msg._buff)
                    self._last_map_msg = og
                    self._map_is_occupancy_grid = True
                    known = sum(1 for v in og.data if v >= 0)
                    total = len(og.data)
                    self._map_known_area_series.append({
                        "sim_time": self._current_sim_time.to_sec(),
                        "known_cells": known,
                        "total_cells": total,
                    })
                except Exception:
                    pass
            elif 'PointCloud2' in msg_type:
                # Store point cloud message directly
                try:
                    from sensor_msgs.msg import PointCloud2
                    pc = PointCloud2()
                    pc.deserialize(msg._buff)
                    self._last_map_msg = pc
                except Exception:
                    pass
            # else: unknown type — not stored as map
        except Exception:
            pass

    def _octomap_cb(self, msg):
        """Track OctoMap binary messages for map growth monitoring."""
        self._octomap_binary_msg = msg
        # Count nodes from binary message (rough estimate)
        try:
            data_len = len(msg.data) if hasattr(msg, 'data') else 0
            # Each octomap node is ~12 bytes serialized
            node_estimate = max(0, data_len // 12)
            self._octomap_node_count_series.append({
                "sim_time": self._current_sim_time.to_sec(),
                "node_estimate": node_estimate,
            })
        except Exception:
            pass

    def _frontier_cb(self, msg):
        """Track frontier point cloud for exploration progress."""
        self._last_frontier_count = msg.width if hasattr(msg, 'width') else 0
        self._frontier_history.append({
            "sim_time": self._current_sim_time.to_sec(),
            "point_count": self._last_frontier_count,
        })

    def _dsv_runtime_cb(self, msg):
        """DSV planner activity heartbeat."""
        self._planner_alive = True
        self._last_planner_activity = wall_now()
        self._last_dsv_runtime = msg.data

    # -----------------------------------------------------------------------
    # Health monitoring
    # -----------------------------------------------------------------------
    def _health_check(self, _event):
        """Periodic health check (1 Hz)."""
        with self._lock:
            now = wall_now()
            sim_now = self._current_sim_time.to_sec()

            # Odometry health
            odom_age = (now - self._last_odom_wall_time).total_seconds()
            self._odom_healthy = (odom_age < self.odom_timeout and
                                  not self._odom_nan_detected)

            # Map health
            map_age = (sim_now - self._last_map_sim_time.to_sec()
                       if self._last_map_sim_time.to_sec() > 0 else 999)
            self._map_healthy = (self._map_count > 0 and map_age < 30.0)

            # Robot fall detection: height drop or extreme roll/pitch
            if len(self._robot_height_history) >= 10 and self._odom_samples:
                avg_height = np.mean(list(self._robot_height_history))
                last_sample = self._odom_samples[-1]
                if avg_height < 0.15:  # Robot on ground
                    self._robot_fall_detected = True
                    self._health_events.append({
                        "wall_time": wall_iso(),
                        "sim_time": sim_now,
                        "event": "ROBOT_FALL",
                        "detail": f"Average height {avg_height:.3f}m below threshold"
                    })
                if abs(last_sample.get("roll", 0)) > 1.2 or abs(last_sample.get("pitch", 0)) > 1.2:
                    self._robot_fall_detected = True
                    self._health_events.append({
                        "wall_time": wall_iso(),
                        "sim_time": sim_now,
                        "event": "ROBOT_TILT",
                        "detail": f"Roll={last_sample['roll']:.2f} Pitch={last_sample['pitch']:.2f}"
                    })

            # Planner liveness (DSV + FALCO)
            planner_age = (now - self._last_planner_activity).total_seconds()
            self._planner_alive = planner_age < 30.0
            falco_age = (now - self._last_falco_activity).total_seconds()
            self._falco_alive = falco_age < 30.0
            bridge_age = (now - self._last_bridge_activity).total_seconds()
            self._bridge_alive = bridge_age < 30.0 or self._cmd_vel_count > 0

            # If we haven't received any cmd_vel yet but FALCO is running,
            # bridge is considered alive
            if not self._bridge_alive and self._fsm_state == 4:
                self._bridge_alive = True

    def _status_log(self, _event):
        """Periodic status report."""
        with self._lock:
            route_metrics = self._route_accumulator.metrics
            rospy.loginfo(
                "[Recorder] Status: sim=%.1f odom=%d goals=%d(raw)/%d(unq) "
                "map=%d octo=%d path=%.1fm route_segments=%d/%d "
                "tf_drops=%d noeff=%d started=%s fsm=%d fall=%s",
                self._current_sim_time.to_sec(),
                self._odom_count,
                self._raw_goal_count,
                len(self._unique_goals),
                self._map_count,
                len(self._octomap_node_count_series),
                route_metrics.route_length_m,
                route_metrics.route_accepted_segments,
                route_metrics.route_rejected_segments,
                self._trajectory_transform_drop_count,
                self._noeff_count,
                self._exploration_started,
                self._fsm_state,
                self._robot_fall_detected,
            )

    # -----------------------------------------------------------------------
    # Completion detection
    # -----------------------------------------------------------------------
    def _check_completion(self, _event):
        """Check if exploration is complete (2 Hz)."""
        if self._exploration_completed or self._shutdown_requested:
            return

        with self._lock:
            sim_now = self._current_sim_time.to_sec()

            # ── Prerequisite: exploration must have started ──
            if not self._exploration_started:
                return

            # ── Method 0: Minimal map validation (diagnostic mode) ──
            if self.minimal_map_validation:
                if (self._map_is_occupancy_grid
                        and self._map_count >= self.stop_after_map_updates):
                    self._complete_exploration(
                        COMPLETION_METHOD_MINIMAL_MAP,
                        f"Minimal map validation: received {self._map_count} "
                        f"OccupancyGrid updates (threshold={self.stop_after_map_updates})")
                    return

            # ── Method 1: DSV native stop signal ──
            if self._dvs_native_stop_received:
                self._complete_exploration(
                    COMPLETION_METHOD_DSV_NATIVE,
                    "DSV native stop signal received")
                return

            # ── Method 2: Absolute timeout ──
            if self._exploration_start_sim_time is not None:
                elapsed_sim = sim_now - self._exploration_start_sim_time.to_sec()
                if elapsed_sim > self.max_sim_time:
                    self._complete_exploration(
                        COMPLETION_METHOD_TIMEOUT,
                        f"Max sim time {self.max_sim_time}s exceeded (elapsed={elapsed_sim:.1f}s)")
                    return

            # ── Method 3: Composite quiet-window detection ──
            if self._check_composite_completion(sim_now):
                return

    def _check_composite_completion(self, sim_now):
        """Check composite completion criteria.

        Returns True if all criteria are met and exploration completes.
        """
        # All prerequisites
        if not self._exploration_started:
            return False
        if not self._odom_healthy:
            return False
        if not self._planner_alive:
            return False
        if not self._map_healthy:
            return False
        if self._robot_fall_detected:
            return False

        # Check: no new unique goal for quiet window
        if self._last_unique_goal_sim_time.to_sec() > 0:
            time_since_last_goal = sim_now - self._last_unique_goal_sim_time.to_sec()
            if time_since_last_goal < self.finish_quiet_time:
                return False
        else:
            # No goals recorded at all — only allow composite completion
            # after the exploration has run for at least finish_quiet_time,
            # confirming the environment is genuinely frontier-less.
            if self._exploration_start_sim_time is not None:
                elapsed = sim_now - self._exploration_start_sim_time.to_sec()
                if elapsed < self.finish_quiet_time:
                    return False
            else:
                return False

        # Check: map growth below threshold
        map_stagnant = self._check_map_stagnant(sim_now)
        if not map_stagnant:
            return False

        # Check: robot motion below threshold
        robot_stationary = self._check_robot_stationary(sim_now)
        if not robot_stationary:
            return False

        # Check: no pending valid goal
        has_pending = self._has_pending_goal()
        if has_pending:
            return False

        # Check: frontiers exhausted
        if len(self._frontier_history) >= 5:
            recent_frontiers = list(self._frontier_history)[-5:]
            avg_frontier = np.mean([f["point_count"] for f in recent_frontiers])
            if avg_frontier > 5:  # Still significant frontiers
                return False

        # All criteria met
        self._complete_exploration(
            COMPLETION_METHOD_COMPOSITE,
            "Composite quiet-window criteria met: "
            f"no_new_goal>{self.finish_quiet_time}s, map_stagnant, robot_stationary, "
            f"no_pending_goal, frontiers_exhausted")
        return True

    def _check_map_stagnant(self, sim_now):
        """Check if map growth has stagnated."""
        if len(self._octomap_node_count_series) < 5:
            return False
        recent = list(self._octomap_node_count_series)[-5:]
        oldest = recent[0]["node_estimate"]
        newest = recent[-1]["node_estimate"]
        if oldest <= 0:
            return False
        growth_ratio = (newest - oldest) / float(oldest)
        return growth_ratio < self.map_growth_threshold

    def _check_robot_stationary(self, sim_now):
        """Check if robot has moved little recently."""
        if len(self._odom_samples) < 50:
            return False
        # Check last ~10 seconds of sim time
        recent = [s for s in self._odom_samples[-50:]
                  if sim_now - s["sim_time"] < 15.0]
        if len(recent) < 5:
            return False
        first = recent[0]
        last = recent[-1]
        dist = math.sqrt((last["x"] - first["x"])**2 + (last["y"] - first["y"])**2)
        return dist < self.robot_motion_threshold

    def _has_pending_goal(self):
        """Check if there's a currently active unreached goal."""
        if not self._unique_goals:
            return False
        last_goal = self._unique_goals[-1]
        if last_goal["status"] in (GOAL_STATUS_PUBLISHED, GOAL_STATUS_REPLACED):
            # Check if robot is close to this goal
            if self._last_position is not None:
                dx = self._last_position[0] - last_goal["x"]
                dy = self._last_position[1] - last_goal["y"]
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < 0.5:
                    last_goal["status"] = GOAL_STATUS_REACHED
                    last_goal["reached"] = True
                    last_goal["reached_sim_time"] = self._current_sim_time.to_sec()
                    last_goal["reached_distance_m"] = dist
                    return False
            return True  # Still has pending goal
        return False

    def _complete_exploration(self, method, reason):
        """Mark exploration as complete."""
        self._exploration_completed = True
        self._completion_method = method
        self._completion_reason = reason
        self._exploration_end_sim_time = self._current_sim_time
        self._exploration_end_wall = wall_now()
        self._completion_sim_time = self._current_sim_time.to_sec()

        rospy.loginfo("[Recorder] === EXPLORATION COMPLETE ===")
        rospy.loginfo("[Recorder] Method: %s", method)
        rospy.loginfo("[Recorder] Reason: %s", reason)
        rospy.loginfo("[Recorder] End sim time: %.3f", self._current_sim_time.to_sec())

        # Signal stop (re-publish for safety)
        try:
            self._stop_repub.publish(Bool(data=True))
        except Exception:
            pass

        # Execute controlled stop
        self._execute_stop_sequence()

    def _execute_stop_sequence(self):
        """Controlled stop: zero velocity, wait for map stability, save results."""
        rospy.loginfo("[Recorder] Starting controlled stop sequence...")

        # Mark all remaining goals as UNREACHED_AT_FINISH
        with self._lock:
            for g in self._unique_goals:
                if g["status"] in (GOAL_STATUS_PUBLISHED, GOAL_STATUS_REPLACED):
                    g["status"] = GOAL_STATUS_UNREACHED_AT_FINISH
            self._reached_goals = sum(1 for g in self._unique_goals if g["reached"])
            self._unreached_goals = len(self._unique_goals) - self._reached_goals

        # Send zero velocity continuously for 2 wall seconds
        zero_twist = Twist()
        self._zero_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        stop_start = wall_now()
        while wall_elapsed(stop_start) < 2.0 and not rospy.is_shutdown():
            try:
                self._zero_vel_pub.publish(zero_twist)
            except Exception:
                pass
            time.sleep(0.1)
        rospy.loginfo("[Recorder] Zero velocity sent for 2s.")

        # Wait for map stability (sim-time based, with wall-clock timeout)
        map_wait_start_sim = self._current_sim_time.to_sec()
        map_wait_start_wall = wall_now()
        rospy.loginfo("[Recorder] Waiting for map stability (%ss sim)...",
                       self.map_stable_wait)
        while (self._current_sim_time.to_sec() - map_wait_start_sim < self.map_stable_wait
               and wall_elapsed(map_wait_start_wall) < 30.0
               and not rospy.is_shutdown()):
            time.sleep(0.2)
        rospy.loginfo("[Recorder] Map stability wait complete.")

        # Save all results
        self._save_all_results()

    # -----------------------------------------------------------------------
    # Result saving
    # -----------------------------------------------------------------------
    def _save_all_results(self):
        """Save all collected data to output directory."""
        rospy.loginfo("[Recorder] Saving all results to %s ...", self.output_dir)
        try:
            self._save_trajectory()
            self._save_goals()
            self._save_map()
            self._save_timing()
            self._save_config()
            self._save_summary()
            self._save_manifest()
            self._save_health_log()
            if HAS_MPL:
                self._save_plot()
            else:
                rospy.logwarn("[Recorder] matplotlib not available; skipping plot.")
            rospy.loginfo("[Recorder] === EXPLORATION_RESULTS_SAVED ===")
        except Exception as e:
            rospy.logerr("[Recorder] Error saving results: %s", e)
            traceback.print_exc()
            try:
                self._save_partial_manifest()
                self._save_summary_error(str(e))
            except Exception:
                pass
            rospy.logwarn("[Recorder] === PARTIAL_RESULTS_SAVED ===")

    def _save_trajectory(self):
        """Save robot trajectory as CSV and YAML."""
        with self._lock:
            samples = list(self._odom_samples)

        if not samples:
            rospy.logwarn("[Recorder] No trajectory data to save.")
            return

        # Preserve callback order, including duplicate or backward timestamps.
        # The shared policy rejects those segments and reports why; silently
        # sorting/deduplicating here would erase the diagnostic evidence.
        saved_samples = samples
        route_metrics = compute_route_length(saved_samples, self._route_policy)
        with self._lock:
            self._final_route_metrics = route_metrics

        csv_path = os.path.join(self.output_dir, "route", "trajectory.csv")
        fieldnames = [
            "index", "sim_time", "wall_time",
            "x", "y", "z",
            "qx", "qy", "qz", "qw",
            "roll", "pitch", "yaw",
            "linear_velocity_x", "linear_velocity_y", "linear_velocity_z",
            "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
            "frame_id", "child_frame_id",
            "source_frame_id", "target_frame_id", "source_topic",
            "transform_applied",
        ]
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(saved_samples)

        # YAML summary
        yaml_path = os.path.join(self.output_dir, "route", "trajectory.yaml")
        with open(yaml_path, "w") as f:
            yaml.dump({
                "source_topic": self.trajectory_pose_topic,
                "total_points": len(saved_samples),
                "total_points_raw": len(samples),
                "trajectory_length_m_2d": route_metrics.route_length_m,
                **route_metrics.to_dict(),
                **self._route_policy.to_dict(),
                "has_nan": self._odom_nan_detected,
                "has_jump": self._odom_jump_detected,
                "frame_id": self.trajectory_target_frame,
                "child_frame_id": (saved_samples[0]["child_frame_id"]
                                   if saved_samples else ""),
                "source_frames": sorted(self._trajectory_source_frames),
                "transform_drop_count": self._trajectory_transform_drop_count,
                "start_sim_time": (saved_samples[0]["sim_time"]
                                   if saved_samples else 0),
                "end_sim_time": (saved_samples[-1]["sim_time"]
                                 if saved_samples else 0),
            }, f)

        metrics_path = os.path.join(self.output_dir, "route", "metrics.yaml")
        with open(metrics_path, "w") as f:
            yaml.dump({
                **route_metrics.to_dict(),
                **self._route_policy.to_dict(),
                "trajectory_source_topic": self.trajectory_pose_topic,
                "trajectory_source_frames": sorted(self._trajectory_source_frames),
                "trajectory_target_frame": self.trajectory_target_frame,
                "transform_drop_count": self._trajectory_transform_drop_count,
            }, f)

        rospy.loginfo(
            "[Recorder] Trajectory saved: %d points, %.1fm, segments=%d/%d",
            len(saved_samples), route_metrics.route_length_m,
            route_metrics.route_accepted_segments,
            route_metrics.route_rejected_segments)

    def _save_goals(self):
        """Save DSV goals as CSV and YAML."""
        with self._lock:
            all_goals = list(self._goals)
            unique_goals = list(self._unique_goals)

        csv_path = os.path.join(self.output_dir, "goals", "goals.csv")
        fieldnames = [
            "goal_index", "received_sim_time", "received_wall_time",
            "x", "y", "z", "frame_id", "source_topic",
            "is_duplicate", "reached", "reached_sim_time",
            "reached_distance_m", "status",
        ]
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(all_goals)

        # Unique goals CSV
        unique_csv = os.path.join(self.output_dir, "goals", "goals_unique.csv")
        with open(unique_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames + ["goal_index_unique"],
                                     extrasaction='ignore')
            writer.writeheader()
            writer.writerows(unique_goals)

        yaml_path = os.path.join(self.output_dir, "goals", "goals.yaml")
        with open(yaml_path, "w") as f:
            yaml.dump({
                "source_topic": self.goal_topic,
                "raw_message_count": len(all_goals),
                "unique_goal_count": len(unique_goals),
                "reached_goal_count": self._reached_goals,
                "unreached_goal_count": self._unreached_goals,
                "dedup_distance_m": self.goal_dedup_distance,
            }, f)

        rospy.loginfo("[Recorder] Goals saved: %d raw, %d unique, %d reached",
                       len(all_goals), len(unique_goals), self._reached_goals)

    def _save_map(self):
        """Save 2D occupancy grid map if available, or generate from OctoMap."""
        if self._last_map_msg is not None and self._map_is_occupancy_grid:
            self._save_occupancy_grid_map()
        else:
            self._save_map_fallback()
        self._save_map_metadata()

    def _save_occupancy_grid_map(self):
        """Save OccupancyGrid as PGM + YAML."""
        try:
            from PIL import Image
        except ImportError:
            rospy.logwarn("[Recorder] PIL not available; cannot save PGM.")
            return

        msg = self._last_map_msg
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Convert data to 2D numpy array
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # OccupancyGrid: 0=free, 100=occupied, -1=unknown
        # PGM: 0=black(occupied), 254=white(free), 205=gray(unknown)
        pgm = np.ones((height, width), dtype=np.uint8) * 205  # unknown
        pgm[data == 0] = 254    # free → white
        pgm[data == 100] = 0    # occupied → black
        pgm = np.flipud(pgm)    # PGM row 0 is top

        pgm_path = os.path.join(self.output_dir, "map", "map.pgm")
        img = Image.fromarray(pgm, mode='L')
        img.save(pgm_path)

        yaml_path = os.path.join(self.output_dir, "map", "map.yaml")
        origin = msg.info.origin
        with open(yaml_path, "w") as f:
            yaml.dump({
                "image": "map.pgm",
                "resolution": resolution,
                "origin": [origin.position.x, origin.position.y, origin.position.z],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }, f)

        # CSV
        csv_path = os.path.join(self.output_dir, "map", "occupancy_grid.csv")
        np.savetxt(csv_path, pgm, fmt="%d", delimiter=",")

        rospy.loginfo("[Recorder] OccupancyGrid map saved: %dx%d @ %.3fm/px",
                       width, height, resolution)

    def _save_map_fallback(self):
        """Generate minimal 2D map from terrain point cloud or trajectory bounds."""
        rospy.logwarn("[Recorder] No OccupancyGrid available; saving minimal map info.")
        pgm_path = os.path.join(self.output_dir, "map", "map.pgm")
        yaml_path = os.path.join(self.output_dir, "map", "map.yaml")

        # Create a minimal placeholder
        default_size = 400
        default_res = 0.1
        try:
            from PIL import Image
            img = Image.new('L', (default_size, default_size), 205)
            img.save(pgm_path)
        except Exception:
            pass

        with open(yaml_path, "w") as f:
            yaml.dump({
                "image": "map.pgm",
                "resolution": default_res,
                "origin": [-20.0, -20.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }, f)

    def _save_map_metadata(self):
        """Save map metadata YAML."""
        placeholder_map_used = (
            not self._map_is_occupancy_grid
            or self._last_map_msg is None
        )
        meta = {
            "map_topic": self.map_topic,
            "map_is_occupancy_grid": self._map_is_occupancy_grid,
            "placeholder_map_used": placeholder_map_used,
            "real_map_received": (not placeholder_map_used
                                  and self._map_count > 0),
            "map_sim_time": (self._last_map_sim_time.to_sec()
                              if self._last_map_sim_time.to_sec() > 0 else 0),
            "map_message_count": self._map_count,
            "map_update_count": self._map_count,
            "octomap_node_samples": len(self._octomap_node_count_series),
        }

        if self._last_map_msg is not None and self._map_is_occupancy_grid:
            msg = self._last_map_msg
            data = np.array(msg.data, dtype=np.int8)
            meta.update({
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "origin_z": msg.info.origin.position.z,
                "occupied_cell_count": int(np.sum(data == 100)),
                "free_cell_count": int(np.sum(data == 0)),
                "unknown_cell_count": int(np.sum(data == -1)),
                "known_area_m2": float(np.sum(data >= 0) * msg.info.resolution**2),
                "occupied_area_m2": float(np.sum(data == 100) * msg.info.resolution**2),
                "free_area_m2": float(np.sum(data == 0) * msg.info.resolution**2),
                "map_frame": msg.header.frame_id if hasattr(msg.header, 'frame_id') else "",
            })

        with open(os.path.join(self.output_dir, "map", "metadata.yaml"), "w") as f:
            yaml.dump(meta, f)

    def _save_timing(self):
        """Save timing data."""
        with self._lock:
            start_sim = self._exploration_start_sim_time
            end_sim = self._exploration_end_sim_time
            start_wall = getattr(self, '_exploration_start_wall', self._start_wall)
            end_wall = getattr(self, '_exploration_end_wall', wall_now())

        sim_start = start_sim.to_sec() if start_sim else 0.0
        sim_end = end_sim.to_sec() if end_sim else self._current_sim_time.to_sec()
        duration_sim = sim_end - sim_start
        duration_wall = (end_wall - start_wall).total_seconds()
        avg_rtf = duration_sim / duration_wall if duration_wall > 0 else 0.0

        timing = {
            "process_start_wall_time": wall_iso(self._process_start_wall),
            "exploration_start_sim_time": sim_start,
            "exploration_start_wall_time": wall_iso(start_wall),
            "exploration_end_sim_time": sim_end,
            "exploration_end_wall_time": wall_iso(end_wall),
            "exploration_duration_sim_sec": duration_sim,
            "exploration_duration_wall_sec": duration_wall,
            "average_rtf": avg_rtf,
            "sim_clock_reset_detected": self._clock_reset_detected,
            "sim_clock_jump_detected": self._clock_jump_detected,
            "clock_sample_count": self._clock_sample_count,
            "completion_method": self._completion_method,
            "completion_reason": self._completion_reason,
            "completion_sim_time": self._completion_sim_time,
            "timing_valid": (not self._clock_reset_detected and
                             duration_sim > 0 and sim_end >= sim_start),
        }

        with open(os.path.join(self.output_dir, "timing", "timing.yaml"), "w") as f:
            yaml.dump(timing, f)

        rospy.loginfo("[Recorder] Timing saved: sim=%.1fs wall=%.1fs RTF=%.2f",
                       duration_sim, duration_wall, avg_rtf)

    def _save_config(self):
        """Save runtime configuration."""
        config = {
            "output_dir": self.output_dir,
            "run_id": self.run_id,
            "max_sim_time": self.max_sim_time,
            "finish_quiet_time": self.finish_quiet_time,
            "map_stable_wait": self.map_stable_wait,
            "goal_dedup_distance": self.goal_dedup_distance,
            "map_growth_threshold": self.map_growth_threshold,
            "robot_motion_threshold": self.robot_motion_threshold,
            "odom_timeout": self.odom_timeout,
            "trajectory_target_frame": self.trajectory_target_frame,
            "route_max_speed_mps": self.route_max_speed_mps,
            "route_max_step_m": self.route_max_step_m,
            "transform_timeout": self.transform_timeout,
            "trajectory_transform_drop_count": self._trajectory_transform_drop_count,
            "trajectory_source_frames": sorted(self._trajectory_source_frames),
            "topic_config": {
                "clock": self.clock_topic,
                "odometry": self.trajectory_pose_topic,
                "trajectory_pose": self.trajectory_pose_topic,
                "goal": self.goal_topic,
                "map": self.map_topic,
                "octomap_binary": self.octomap_binary_topic,
                "cmd_vel": self.cmd_vel_topic,
                "stop_signal": self.stop_signal_topic,
                "start_signal": self.start_signal_topic,
                "frontier": self.frontier_topic,
            },
            "completion_config": {
                "method_priority": [
                    "dsv_native_stop_signal",
                    "composite_quiet_window",
                    "exploration_timeout",
                ],
                "quiet_window_criteria": {
                    "no_new_unique_goal_sim_sec": self.finish_quiet_time,
                    "map_growth_below_threshold": self.map_growth_threshold,
                    "robot_motion_below_threshold": self.robot_motion_threshold,
                    "require_odometry_healthy": True,
                    "require_planner_alive": True,
                    "require_map_healthy": True,
                    "require_no_robot_fall": True,
                    "require_frontiers_exhausted": True,
                    "require_no_pending_goal": True,
                },
            },
        }
        with open(os.path.join(self.output_dir, "config", "recorder_config.yaml"), "w") as f:
            yaml.dump(config, f)

    def _save_runtime_config(self):
        """Save initial runtime environment info."""
        env_info = {
            "run_id": self.run_id,
            "output_dir": self.output_dir,
            "start_wall_time": wall_iso(self._start_wall),
            "ros_python": sys.executable,
            "argv": sys.argv,
        }
        with open(os.path.join(self.output_dir, "config", "runtime_env.txt"), "w") as f:
            for k, v in env_info.items():
                f.write(f"{k}: {v}\n")

    def _save_manifest(self):
        """Save file manifest of all output artifacts."""
        manifest = {
            "output_dir": self.output_dir,
            "run_id": self.run_id,
            "generated_at": wall_iso(),
            "files": {},
        }

        for root, dirs, files in os.walk(self.output_dir):
            for fname in files:
                fpath = os.path.join(root, fname)
                rel = os.path.relpath(fpath, self.output_dir)
                size = os.path.getsize(fpath)
                manifest["files"][rel] = {"size_bytes": size}

        with open(os.path.join(self.output_dir, "manifest.yaml"), "w") as f:
            yaml.dump(manifest, f)

    def _save_partial_manifest(self):
        """Save manifest even on partial failure."""
        try:
            self._save_manifest()
        except Exception:
            pass

    def _save_summary(self):
        """Generate summary.md."""
        with self._lock:
            start_sim = (self._exploration_start_sim_time.to_sec()
                         if self._exploration_start_sim_time else 0)
            end_sim = (self._exploration_end_sim_time.to_sec()
                       if self._exploration_end_sim_time else self._current_sim_time.to_sec())
            start_wall = getattr(self, '_exploration_start_wall', self._start_wall)
            end_wall = getattr(self, '_exploration_end_wall', wall_now())

        duration_sim = end_sim - start_sim
        duration_wall = (end_wall - start_wall).total_seconds()
        avg_rtf = duration_sim / duration_wall if duration_wall > 0 else 0.0

        verdict = self._determine_verdict()
        with self._lock:
            route_metrics = (self._final_route_metrics
                             or self._route_accumulator.metrics)
        reject_reasons = ", ".join(
            f"{reason}={count}"
            for reason, count in route_metrics.route_reject_reasons.items())

        lines = [
            f"# Exploration Run Summary",
            f"",
            f"## Run Info",
            f"- **Run ID**: {self.run_id}",
            f"- **Output Directory**: {self.output_dir}",
            f"",
            f"## Timing",
            f"- **Start wall time**: {wall_iso(start_wall)}",
            f"- **End wall time**: {wall_iso(end_wall)}",
            f"- **Exploration start sim time**: {start_sim:.3f}",
            f"- **Exploration end sim time**: {end_sim:.3f}",
            f"- **Exploration duration sim sec**: {duration_sim:.1f}",
            f"- **Exploration duration wall sec**: {duration_wall:.1f}",
            f"- **Average RTF**: {avg_rtf:.4f}",
            f"",
            f"## Completion",
            f"- **Detection method**: {self._completion_method or 'N/A'}",
            f"- **Reason**: {self._completion_reason or 'N/A'}",
            f"",
            f"## Topics",
            f"- **Map topic**: {self.map_topic}",
            f"- **Trajectory pose topic**: {self.trajectory_pose_topic}",
            f"- **Trajectory target frame**: {self.trajectory_target_frame}",
            f"- **Goal topic**: {self.goal_topic}",
            f"",
            f"## Map",
            f"- **Map message count**: {self._map_count}",
            f"- **Map is OccupancyGrid**: {self._map_is_occupancy_grid}",
            f"",
            f"## Trajectory",
            f"- **Total points**: {self._odom_count}",
            f"- **Route length (2D)**: {route_metrics.route_length_m:.2f} m",
            f"- **Route policy**: finite map-frame poses, increasing time, "
            f"speed <= {self.route_max_speed_mps:.2f} m/s, "
            f"step <= {self.route_max_step_m:.2f} m",
            f"- **Route total segments**: {route_metrics.route_total_segments}",
            f"- **Route accepted segments**: {route_metrics.route_accepted_segments}",
            f"- **Route rejected segments**: {route_metrics.route_rejected_segments}",
            f"- **Route reject reasons**: {reject_reasons}",
            f"- **Trajectory source frames**: "
            f"{', '.join(sorted(self._trajectory_source_frames)) or 'none'}",
            f"- **Transform drops**: {self._trajectory_transform_drop_count}",
            f"",
            f"## Goals",
            f"- **Raw goal messages**: {self._raw_goal_count}",
            f"- **Unique goals**: {len(self._unique_goals)}",
            f"- **Reached goals**: {self._reached_goals}",
            f"- **Unreached goals**: {self._unreached_goals}",
            f"",
            f"## Health",
            f"- **NoEff count**: {self._noeff_count}",
            f"- **NoEff source**: cmd_vel_stall_heuristic (navigation_enabled && fsm_state==4 && cmd_vel≈0; "
            f"NOT a hardware NoEff diagnostic)",
            f"- **Robot fall detected**: {self._robot_fall_detected}",
            f"- **Clock reset detected**: {self._clock_reset_detected}",
            f"- **Odom NaN detected**: {self._odom_nan_detected}",
            f"- **Trotting (state=4) achieved**: {self._trotting_achieved}",
            f"- **FSM state**: {self._fsm_state}",
            f"",
            f"## Verdict",
            f"- **Final verdict**: {verdict}",
        ]

        with open(os.path.join(self.output_dir, "summary.md"), "w") as f:
            f.write("\n".join(lines) + "\n")

    def _save_summary_error(self, error_msg):
        """Save summary even when save fails."""
        try:
            lines = [
                f"# Exploration Run Summary (Error)",
                f"- **Run ID**: {self.run_id}",
                f"- **Error**: {error_msg}",
                f"- **Verdict**: {VERDICT_FAILURE}",
                f"- **PARTIAL_RESULTS_SAVED**: true",
            ]
            with open(os.path.join(self.output_dir, "summary.md"), "w") as f:
                f.write("\n".join(lines) + "\n")
        except Exception:
            pass

    def _save_health_log(self):
        """Save health events log."""
        with open(os.path.join(self.output_dir, "logs", "health_events.log"), "w") as f:
            for event in self._health_events:
                f.write(json.dumps(event) + "\n")

    def _save_plot(self):
        """Generate map_route_goals.png overlay plot."""
        fig, ax = plt.subplots(figsize=(12, 10))

        # Plot map if available
        if self._last_map_msg is not None and self._map_is_occupancy_grid:
            try:
                msg = self._last_map_msg
                width = msg.info.width
                height = msg.info.height
                res = msg.info.resolution
                origin = msg.info.origin.position
                data = np.array(msg.data, dtype=np.int8).reshape((height, width))

                # Create RGBA image for map
                map_img = np.zeros((height, width, 4), dtype=np.float32)
                map_img[data == -1, :] = [0.5, 0.5, 0.5, 1.0]    # unknown → gray
                map_img[data == 0, :] = [1.0, 1.0, 1.0, 1.0]      # free → white
                map_img[data == 100, :] = [0.0, 0.0, 0.0, 1.0]    # occupied → black

                extent = [
                    origin.x,
                    origin.x + width * res,
                    origin.y,
                    origin.y + height * res,
                ]
                ax.imshow(map_img, extent=extent, origin='lower', aspect='equal')
            except Exception as e:
                rospy.logwarn("[Recorder] Plot: cannot render map: %s", e)
                ax.set_xlim(-20, 20)
                ax.set_ylim(-20, 20)
        else:
            ax.set_xlim(-20, 20)
            ax.set_ylim(-20, 20)
            ax.set_facecolor('lightgray')

        # Plot trajectory
        with self._lock:
            traj = list(self._odom_samples)
        if traj:
            xs = [s["x"] for s in traj]
            ys = [s["y"] for s in traj]
            ax.plot(xs, ys, 'b-', linewidth=1.0, alpha=0.7, label='Robot trajectory')
            ax.plot(xs[0], ys[0], 'go', markersize=8, label='Start')
            ax.plot(xs[-1], ys[-1], 'ro', markersize=8, label='End')

        # Plot goals
        with self._lock:
            goals = list(self._unique_goals)
        if goals:
            reached_x = [g["x"] for g in goals if g["reached"]]
            reached_y = [g["y"] for g in goals if g["reached"]]
            unreached_x = [g["x"] for g in goals if not g["reached"]]
            unreached_y = [g["y"] for g in goals if not g["reached"]]

            if reached_x:
                ax.scatter(reached_x, reached_y, c='green', marker='o', s=30,
                          alpha=0.6, label='Reached goals')
            if unreached_x:
                ax.scatter(unreached_x, unreached_y, c='red', marker='x', s=40,
                          alpha=0.6, label='Unreached goals')

            # Goal indices
            for g in goals:
                label = str(g.get("goal_index_unique", g.get("goal_index", "")))
                ax.annotate(label, (g["x"], g["y"]), fontsize=6, alpha=0.7)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'Exploration Map + Route + Goals — {self.run_id}')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        plot_path = os.path.join(self.output_dir, "plots", "map_route_goals.png")
        fig.savefig(plot_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        rospy.loginfo("[Recorder] Plot saved: %s", plot_path)

    def _determine_verdict(self):
        if self._completion_method == COMPLETION_METHOD_TIMEOUT:
            return VERDICT_TIMEOUT
        if self._robot_fall_detected:
            return VERDICT_FAILURE
        if self._clock_reset_detected:
            return VERDICT_FAILURE
        if self._odom_nan_detected:
            return VERDICT_FAILURE
        if self._completion_method == COMPLETION_METHOD_MINIMAL_MAP:
            return VERDICT_MINIMAL_MAP
        if self._completion_method in (COMPLETION_METHOD_DSV_NATIVE,
                                        COMPLETION_METHOD_COMPOSITE):
            return VERDICT_COMPLETE
        return VERDICT_PARTIAL

    # -----------------------------------------------------------------------
    # Shutdown
    # -----------------------------------------------------------------------
    def _on_shutdown(self):
        """Handle ROS shutdown."""
        if self._exploration_completed:
            return  # already saved
        rospy.loginfo("[Recorder] Shutdown requested — saving partial results...")
        self._shutdown_requested = True
        with self._lock:
            if self._exploration_end_sim_time is None:
                self._exploration_end_sim_time = self._current_sim_time
            if not hasattr(self, '_exploration_end_wall'):
                self._exploration_end_wall = wall_now()
        try:
            self._save_all_results()
        except Exception:
            rospy.logerr("[Recorder] Could not save results on shutdown.")
            self._save_partial_manifest()

    # -----------------------------------------------------------------------
    # Utility
    # -----------------------------------------------------------------------
    @staticmethod
    def _quat_to_rpy(orient):
        """Convert quaternion to roll, pitch, yaw."""
        x, y, z, w = orient.x, orient.y, orient.z, orient.w
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
def main():
    rospy.init_node("exploration_result_recorder", anonymous=False)

    # Validate required params
    output_dir = rospy.get_param("~output_dir", "")
    if not output_dir:
        rospy.logfatal("[Recorder] ~output_dir is required.")
        sys.exit(1)

    rospy.loginfo("[Recorder] Starting Exploration Result Recorder.")
    rospy.loginfo("[Recorder] Output: %s", output_dir)

    recorder = ExplorationResultRecorder()

    try:
        # Spin until exploration completes
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not recorder._exploration_completed:
            rate.sleep()

        # After completion, extra time for final data
        rospy.loginfo("[Recorder] Exploration recording phase complete. "
                      "Waiting for final data flush...")
        rospy.sleep(rospy.Duration(3.0))
    except rospy.ROSInterruptException:
        # rospy.Rate.sleep raises during an ordinary rosnode/roslaunch stop.
        # The registered shutdown callback has already persisted partial data.
        pass

    rospy.loginfo("[Recorder] Recording finished.")


if __name__ == "__main__":
    main()
