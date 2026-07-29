#!/usr/bin/env python3
"""Tests for recorder target-frame normalization and transform drops."""

import importlib.util
from pathlib import Path
import sys
import threading
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rospy
import tf2_ros


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))
SCRIPT = PACKAGE / "scripts" / "exploration_result_recorder.py"
SPEC = importlib.util.spec_from_file_location("exploration_result_recorder", SCRIPT)
RECORDER_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RECORDER_MODULE)


class FakeBuffer:
    def __init__(self, output=None, error=None):
        self.output = output
        self.error = error

    def transform(self, _pose, _target, _timeout):
        if self.error is not None:
            raise self.error
        return self.output


def odometry(frame="map", stamp=1.0, x=0.0):
    message = Odometry()
    message.header.frame_id = frame
    message.header.stamp = rospy.Time.from_sec(stamp)
    message.child_frame_id = "body"
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.0
    return message


def transformed_pose(x=1.0):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.from_sec(1.0)
    pose.pose.position.x = x
    pose.pose.orientation.w = 1.0
    return pose


def bare_recorder(buffer):
    recorder = RECORDER_MODULE.ExplorationResultRecorder.__new__(
        RECORDER_MODULE.ExplorationResultRecorder)
    recorder._lock = threading.RLock()
    recorder.trajectory_target_frame = "map"
    recorder.trajectory_pose_topic = "/navigation/state_estimation"
    recorder.transform_timeout = 0.05
    recorder._tf_buffer = buffer
    recorder._trajectory_transform_drop_count = 0
    recorder._trajectory_source_frames = set()
    recorder._sim_time_started = True
    recorder._odom_nan_detected = False
    recorder._odom_jump_detected = False
    recorder._health_events = []
    recorder._robot_height_history = []
    recorder._last_position = None
    recorder._route_policy = RECORDER_MODULE.RoutePolicy()
    recorder._route_accumulator = RECORDER_MODULE.RouteAccumulator(
        recorder._route_policy)
    recorder._odom_samples = []
    recorder._odom_count = 0
    recorder._last_odom_sim_time = rospy.Time(0)
    recorder._last_odom_wall_time = RECORDER_MODULE.wall_now()
    return recorder


class RecorderFrameTest(unittest.TestCase):
    def test_map_pose_is_recorded_without_transform(self):
        recorder = bare_recorder(FakeBuffer())
        recorder._odom_cb(odometry(frame="map", x=0.5))
        self.assertEqual(len(recorder._odom_samples), 1)
        self.assertEqual(recorder._odom_samples[0]["frame_id"], "map")
        self.assertEqual(recorder._odom_samples[0]["source_frame_id"], "map")
        self.assertFalse(recorder._odom_samples[0]["transform_applied"])

    def test_camera_init_pose_is_transformed_and_recorded_as_map(self):
        recorder = bare_recorder(FakeBuffer(output=transformed_pose(x=4.0)))
        recorder._odom_cb(odometry(frame="camera_init", x=1.0))
        self.assertEqual(len(recorder._odom_samples), 1)
        self.assertEqual(recorder._odom_samples[0]["x"], 4.0)
        self.assertEqual(recorder._odom_samples[0]["frame_id"], "map")
        self.assertEqual(recorder._odom_samples[0]["source_frame_id"],
                         "camera_init")
        self.assertTrue(recorder._odom_samples[0]["transform_applied"])

    def test_missing_transform_drops_sample_and_increments_counter(self):
        recorder = bare_recorder(
            FakeBuffer(error=tf2_ros.LookupException("missing")))
        with mock.patch.object(RECORDER_MODULE.rospy, "logwarn_throttle"):
            recorder._odom_cb(odometry(frame="camera_init"))
        self.assertEqual(recorder._odom_samples, [])
        self.assertEqual(recorder._trajectory_transform_drop_count, 1)

    def test_source_frame_change_never_mixes_recorded_frames(self):
        recorder = bare_recorder(FakeBuffer(output=transformed_pose(x=2.0)))
        recorder._odom_cb(odometry(frame="map", stamp=1.0, x=1.0))
        recorder._odom_cb(odometry(frame="camera_init", stamp=2.0, x=2.0))
        self.assertEqual(
            {row["frame_id"] for row in recorder._odom_samples}, {"map"})
        self.assertEqual(recorder._trajectory_source_frames,
                         {"map", "camera_init"})


if __name__ == "__main__":
    unittest.main()
