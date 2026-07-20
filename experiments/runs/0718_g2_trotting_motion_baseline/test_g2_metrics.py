#!/usr/bin/env python3
"""Unit tests for G2 metric helpers."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g2_metrics as metrics


class G2MetricsTest(unittest.TestCase):
    def test_world_to_body_velocity_uses_yaw(self):
        body_vx, body_vy = metrics.world_to_body_velocity(0.0, 1.0, math.pi / 2.0)
        self.assertAlmostEqual(body_vx, 1.0, places=6)
        self.assertAlmostEqual(body_vy, 0.0, places=6)

    def test_wrap_angle(self):
        self.assertAlmostEqual(metrics.wrap_angle_rad(3.5), 3.5 - 2.0 * math.pi)
        self.assertAlmostEqual(metrics.wrap_angle_rad(-3.5), -3.5 + 2.0 * math.pi)

    def test_rows_in_window_is_inclusive(self):
        rows = [{"sim_time": 1.0}, {"sim_time": 2.0}, {"sim_time": 3.0}]
        self.assertEqual(metrics.rows_in_window(rows, 1.0, 2.0), rows[:2])

    def test_stop_time_detection(self):
        rows = [
            {"sim_time": 10.0, "body_vx": 0.20, "body_vy": 0.0},
            {"sim_time": 10.5, "body_vx": 0.04, "body_vy": 0.0},
        ]
        self.assertEqual(metrics.stop_time_to_threshold(rows, 10.0, 0.05), 0.5)
        self.assertIsNone(metrics.stop_time_to_threshold(rows, 10.0, 0.01))

    def test_lateral_drift_ratio(self):
        self.assertAlmostEqual(metrics.lateral_drift_ratio(2.0, -0.1), 0.05)
        self.assertGreater(metrics.lateral_drift_ratio(0.0, 0.1), 1e6)

    def test_aggregate_valid_trials_filters_invalid(self):
        result = metrics.aggregate_valid_trials(
            [
                {"trial_result": "PASS", "steady_mean_vx": 0.1},
                {"trial_result": "FAIL", "steady_mean_vx": 0.3},
                {"trial_result": "INVALID", "steady_mean_vx": 9.0},
            ],
            "steady_mean_vx",
        )
        self.assertAlmostEqual(result["median"], 0.2)


if __name__ == "__main__":
    unittest.main()
