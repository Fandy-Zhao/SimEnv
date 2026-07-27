#!/usr/bin/env python3
"""Unit tests for navigation odometry frame composition."""

import importlib.util
import math
from pathlib import Path
import unittest

from geometry_msgs.msg import Point, Quaternion


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "odometry_to_map.py"
SPEC = importlib.util.spec_from_file_location("odometry_to_map", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class OdometryTransformTest(unittest.TestCase):
    def test_translates_identity_pose(self):
        position, rotation = MODULE.compose_pose(
            (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0),
            Point(0.5, -0.5, 1.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        self.assertEqual(position, (1.5, 1.5, 4.0))
        self.assertEqual(tuple(rotation), (0.0, 0.0, 0.0, 1.0))

    def test_rotates_position_and_orientation(self):
        half = math.sqrt(0.5)
        position, rotation = MODULE.compose_pose(
            (0.0, 0.0, 0.0), (0.0, 0.0, half, half),
            Point(1.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(position[0], 0.0, places=6)
        self.assertAlmostEqual(position[1], 1.0, places=6)
        self.assertAlmostEqual(rotation[2], half, places=6)
        self.assertAlmostEqual(rotation[3], half, places=6)


if __name__ == "__main__":
    unittest.main()
