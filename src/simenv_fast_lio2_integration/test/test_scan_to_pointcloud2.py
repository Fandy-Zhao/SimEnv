#!/usr/bin/env python3
"""Regression checks for adapter point-coordinate and frame semantics."""

import ast
import importlib.util
import math
import os
import sys
import types
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ADAPTER = os.path.join(PACKAGE_ROOT, "scripts", "scan_to_pointcloud2.py")


def _load_rotation_builder():
    rospy = types.ModuleType("rospy")
    rospy.get_param = lambda *_args, **_kwargs: None
    rospy.Publisher = object
    rospy.Subscriber = object
    rospy.init_node = lambda *_args, **_kwargs: None
    rospy.spin = lambda: None
    sys.modules.setdefault("rospy", rospy)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.PointCloud = object
    sensor_msgs_msg.PointCloud2 = object
    sensor_msgs_msg.PointField = object
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Header = object
    std_msgs.msg = std_msgs_msg
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    spec = importlib.util.spec_from_file_location("scan_to_pointcloud2", ADAPTER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._build_rotation


class ScanToPointCloud2Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build_rotation = staticmethod(_load_rotation_builder())
        with open(ADAPTER, encoding="utf-8") as source_file:
            cls.source = source_file.read()
        ast.parse(cls.source)

    def test_zero_rotation_preserves_unit_vectors(self):
        for axis in ("x", "y"):
            rotation = self.build_rotation(axis, 0.0)
            for vector in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
                self.assertEqual(rotation(*vector), vector)

    def test_optional_rotation_order_is_y_then_x(self):
        rotate_y = self.build_rotation("y", -90.0)
        rotate_x = self.build_rotation("x", 180.0)
        result = rotate_x(*rotate_y(1.0, 0.0, 0.0))
        self.assertTrue(all(math.isclose(actual, expected, abs_tol=1e-9)
                            for actual, expected in zip(result, (0.0, 0.0, -1.0))))

    def test_default_and_header_contract_are_explicit(self):
        self.assertIn('get_param("~rotation_y_deg", 0.0)', self.source)
        self.assertIn('get_param("~rotation_x_deg", 0.0)', self.source)
        self.assertIn('get_param("~rotated_frame_id", "")', self.source)
        self.assertIn("header = copy.copy(msg.header)", self.source)
        self.assertIn("rotated_frame_id is required", self.source)


if __name__ == "__main__":
    unittest.main()
