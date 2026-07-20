#!/usr/bin/env python3
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import runtime_metrics as metrics


class RuntimeMetricsTest(unittest.TestCase):
    def test_quaternion_tilt_uses_body_z_axis(self):
        self.assertAlmostEqual(metrics.quaternion_tilt_deg(0.0, 0.0, 0.0, 1.0), 0.0)
        qx = math.sin(math.pi / 4.0)
        qw = math.cos(math.pi / 4.0)
        self.assertAlmostEqual(metrics.quaternion_tilt_deg(qx, 0.0, 0.0, qw), 90.0, places=6)

    def test_forward_projection_uses_initial_yaw(self):
        forward, lateral = metrics.world_to_initial_body(0.0, 1.0, math.pi / 2.0)
        self.assertAlmostEqual(forward, 1.0)
        self.assertAlmostEqual(lateral, 0.0, places=6)

    def test_fixedstand_classification_flags_tilt(self):
        summary = {"valid": True, "min_base_height": 0.3, "max_tilt_deg": 50.0, "duration_tilt_over_30_deg": 0.0}
        self.assertEqual(metrics.classify_fixedstand(summary), "FAIL_ATTITUDE")


if __name__ == "__main__":
    unittest.main()
