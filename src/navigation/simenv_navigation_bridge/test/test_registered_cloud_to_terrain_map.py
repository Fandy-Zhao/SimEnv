#!/usr/bin/env python3
"""Focused tests for terrain samples consumed by DSV."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = (Path(__file__).resolve().parents[1] / "scripts" /
          "registered_cloud_to_terrain_map.py")
SPEC = importlib.util.spec_from_file_location("registered_cloud_to_terrain_map", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def filter_points(points, **overrides):
    params = {
        "odom": (0.0, 0.0, 0.20),
        "local_radius": 15.0,
        "robot_self_filter_radius": 0.32,
        "min_relative_z": -0.35,
        "max_relative_z": 1.0,
        "voxel_size": 0.12,
    }
    params.update(overrides)
    return MODULE.filter_terrain_points(points, **params)


class TerrainFilterTest(unittest.TestCase):
    def test_preserves_floor_and_obstacle_samples(self):
        points = [(1.0, 0.0, 0.0), (2.0, 0.0, 0.35)]
        self.assertEqual(filter_points(points), points)

    def test_rejects_out_of_height_radius_and_nonfinite_samples(self):
        points = [
            (1.0, 0.0, -0.20),
            (1.0, 0.0, 1.30),
            (16.0, 0.0, 0.0),
            (0.1, 0.0, 0.0),
            (float("nan"), 1.0, 0.0),
        ]
        self.assertEqual(filter_points(points), [])

    def test_deduplicates_voxels_without_dropping_distinct_floor_cells(self):
        points = [(1.00, 0.0, 0.0), (1.01, 0.0, 0.0), (1.20, 0.0, 0.0)]
        self.assertEqual(filter_points(points), [points[0], points[2]])


if __name__ == "__main__":
    unittest.main()
