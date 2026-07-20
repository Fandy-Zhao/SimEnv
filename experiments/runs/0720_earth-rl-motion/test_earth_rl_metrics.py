#!/usr/bin/env python3
"""Unit tests for earth RL motion metric helpers."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import earth_rl_metrics as metrics


class EarthRlMetricsTest(unittest.TestCase):
    def test_body_forward_displacement_uses_initial_yaw(self):
        rows = [
            {"sim_time": 0.0, "x": 0.0, "y": 0.0, "z": 0.4, "yaw": math.pi / 2.0,
             "roll": 0.0, "pitch": 0.0, "body_vx": 0.0, "body_vy": 0.0, "body_wz": 0.0},
            {"sim_time": 1.0, "x": 0.0, "y": 1.0, "z": 0.4, "yaw": math.pi / 2.0,
             "roll": 0.0, "pitch": 0.0, "body_vx": 1.0, "body_vy": 0.0, "body_wz": 0.0},
        ]
        summary = metrics.summarize_motion(rows)
        self.assertTrue(summary["valid"])
        self.assertAlmostEqual(summary["forward_displacement"], 1.0)
        self.assertAlmostEqual(summary["lateral_displacement"], 0.0, places=6)

    def test_classify_detects_forward_response_failure(self):
        summary = {
            "valid": True,
            "min_z": 0.3,
            "max_roll_deg": 0.0,
            "max_pitch_deg": 0.0,
            "forward_displacement": -0.1,
        }
        self.assertEqual(metrics.classify(summary, requested_state=6, command_vx=0.05), "FAIL_NO_FORWARD_RESPONSE")

    def test_classify_detects_fall(self):
        summary = {
            "valid": True,
            "min_z": 0.05,
            "max_roll_deg": 0.0,
            "max_pitch_deg": 0.0,
            "forward_displacement": 0.0,
        }
        self.assertEqual(metrics.classify(summary, requested_state=2, command_vx=0.0), "FAIL_FALL")


if __name__ == "__main__":
    unittest.main()
