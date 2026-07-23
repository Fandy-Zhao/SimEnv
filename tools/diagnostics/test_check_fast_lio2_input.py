#!/usr/bin/env python3
import math
import os
import struct
import sys
import types
import unittest


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

import check_fast_lio2_input as checker


class Field:
    FLOAT32 = checker.FLOAT32

    def __init__(self, name, offset, datatype=None):
        self.name = name
        self.offset = offset
        self.datatype = checker.FLOAT32 if datatype is None else datatype


class Stamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


class Header:
    def __init__(self, stamp=1.0, frame_id="laser_livox"):
        self.stamp = Stamp(stamp)
        self.frame_id = frame_id


class Cloud:
    def __init__(self, points, stamp=1.0):
        self.header = Header(stamp)
        self.fields = [
            Field("x", 0),
            Field("y", 4),
            Field("z", 8),
            Field("intensity", 12),
        ]
        self.point_step = 16
        self.row_step = 16 * len(points)
        self.width = len(points)
        self.height = 1
        self.is_bigendian = False
        self.data = b"".join(struct.pack("<ffff", x, y, z, 1.0) for x, y, z in points)


class Args:
    pointcloud_topic = "/scan_pointcloud2"
    imu_topic = "/trunk_imu"
    odometry_topic = "/Odometry"
    cloud_registered_topic = "/cloud_registered"
    duration = 30.0
    max_stored_frames = 400
    min_frames = 2
    min_imu_stamps = 2
    blind = 0.5
    max_ros_gap = 2.0
    max_wall_gap = 5.0


class FakeRospy:
    def Subscriber(self, *_args, **_kwargs):
        return object()

    def Rate(self, _hz):
        return types.SimpleNamespace(sleep=lambda: None)

    def is_shutdown(self):
        return False


class CheckFastLio2InputTest(unittest.TestCase):
    def test_cloud_stats_count_finite_and_blind_points(self):
        cloud = Cloud([(1.0, 0.0, 0.0), (math.nan, 0.0, 0.0), (0.1, 0.0, 0.0)])
        stats = checker.summarize_cloud(cloud, blind=0.5, wall_time=10.0)
        self.assertEqual(stats.total_points, 3)
        self.assertEqual(stats.finite_xyz_count, 2)
        self.assertEqual(stats.nonzero_xyz_count, 2)
        self.assertEqual(stats.range_above_blind_count, 1)
        self.assertEqual(stats.frame_id, "laser_livox")

    def test_duplicate_stamp_returns_timestamp_error(self):
        monitor = checker.FastLio2InputMonitor(Args(), FakeRospy(), object, object, object)
        monitor.cloud_cb(Cloud([(1.0, 0.0, 0.0)], stamp=1.0))
        monitor.cloud_cb(Cloud([(1.0, 0.0, 0.0)], stamp=1.0))
        monitor.imu_cb(types.SimpleNamespace(header=Header(1.0)))
        monitor.imu_cb(types.SimpleNamespace(header=Header(1.1)))
        monitor.odom_cb(None)
        monitor.registered_cb(None)
        code, reason = monitor.verdict()
        self.assertEqual(code, 2)
        self.assertIn("duplicate", reason)

    def test_all_inside_blind_returns_effective_point_error(self):
        monitor = checker.FastLio2InputMonitor(Args(), FakeRospy(), object, object, object)
        monitor.cloud_cb(Cloud([(0.1, 0.0, 0.0)], stamp=1.0))
        monitor.cloud_cb(Cloud([(0.2, 0.0, 0.0)], stamp=1.1))
        monitor.imu_cb(types.SimpleNamespace(header=Header(1.0)))
        monitor.imu_cb(types.SimpleNamespace(header=Header(1.1)))
        monitor.odom_cb(None)
        monitor.registered_cb(None)
        code, reason = monitor.verdict()
        self.assertEqual(code, 3)
        self.assertIn("blind", reason)

    def test_missing_fast_lio_outputs_returns_output_error(self):
        monitor = checker.FastLio2InputMonitor(Args(), FakeRospy(), object, object, object)
        monitor.cloud_cb(Cloud([(1.0, 0.0, 0.0)], stamp=1.0))
        monitor.cloud_cb(Cloud([(1.0, 0.0, 0.0)], stamp=1.1))
        monitor.imu_cb(types.SimpleNamespace(header=Header(1.0)))
        monitor.imu_cb(types.SimpleNamespace(header=Header(1.1)))
        code, reason = monitor.verdict()
        self.assertEqual(code, 5)
        self.assertIn("absent", reason)


if __name__ == "__main__":
    unittest.main()
