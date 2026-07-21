#!/usr/bin/env python3
"""Unit tests for rl_fast_metrics pure helpers."""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import rl_fast_metrics as m


class EvaluationWindowTest(unittest.TestCase):
    def test_select_excludes_grace_period(self):
        rows = [
            {"sim_time": 0.0, "z": 0.3},
            {"sim_time": 0.5, "z": 0.3},
            {"sim_time": 1.0, "z": 0.3},
            {"sim_time": 1.5, "z": 0.3},
            {"sim_time": 2.0, "z": 0.3},
        ]
        selected = m.select_evaluation_window(rows, grace_period=1.0)
        times = [r["sim_time"] for r in selected]
        self.assertEqual(times, [1.5, 2.0])

    def test_select_empty_on_short_data(self):
        self.assertEqual(m.select_evaluation_window([], 1.0), [])
        self.assertEqual(m.select_evaluation_window([{"sim_time": 0.0}], 1.0), [])

    def test_win_start_index(self):
        rows = [{"sim_time": t} for t in [0.0, 0.5, 1.0, 1.5, 2.0]]
        self.assertEqual(m.win_start_index(rows, 1.0), 3)
        self.assertEqual(m.win_start_index(rows, 10.0), 5)
        self.assertEqual(m.win_start_index([], 1.0), 0)


class DurationBelowThresholdTest(unittest.TestCase):
    def test_duration_below(self):
        rows = [
            {"sim_time": 0.0, "z": 0.3},
            {"sim_time": 1.0, "z": 0.1},
            {"sim_time": 2.0, "z": 0.1},
            {"sim_time": 3.0, "z": 0.3},
        ]
        dur = m.duration_below_threshold(rows, "z", 0.12)
        self.assertAlmostEqual(dur, 1.2)

    def test_duration_below_none(self):
        rows = [
            {"sim_time": 0.0, "z": 0.3},
            {"sim_time": 1.0, "z": 0.3},
            {"sim_time": 2.0, "z": 0.3},
        ]
        dur = m.duration_below_threshold(rows, "z", 0.12)
        self.assertAlmostEqual(dur, 0.0)

    def test_duration_above(self):
        rows = [
            {"sim_time": 0.0, "tilt_deg": 10.0},
            {"sim_time": 1.0, "tilt_deg": 50.0},
            {"sim_time": 2.0, "tilt_deg": 50.0},
            {"sim_time": 3.0, "tilt_deg": 10.0},
        ]
        dur = m.duration_above_threshold(rows, "tilt_deg", 30.0)
        self.assertAlmostEqual(dur, 2.0)


class VerdictPriorityTest(unittest.TestCase):
    def test_worst_verdict_picks_highest_priority(self):
        verdicts = ["PASS", "FAIL_TILT", "BLOCKED"]
        self.assertEqual(m.worst_verdict(verdicts), "FAIL_TILT")

    def test_worst_verdict_clock_stall_beats_fall(self):
        self.assertEqual(m.worst_verdict(["FAIL_TILT", "FAIL_ROS_CLOCK_PUBLISH"]), "FAIL_ROS_CLOCK_PUBLISH")

    def test_worst_verdict_empty(self):
        self.assertEqual(m.worst_verdict([]), "UNKNOWN")

    def test_verdict_priority_lower_is_worse(self):
        self.assertLess(m.verdict_priority("FAIL_ROS_CLOCK_PUBLISH"), m.verdict_priority("PASS"))
        self.assertEqual(m.verdict_priority("NO_SUCH_VERDICT"), 99)

    def test_verdict_collector(self):
        vc = m.VerdictCollector()
        vc.add("PASS")
        vc.add("FAIL_COMMAND_OUTPUT")
        vc.add("BLOCKED")
        self.assertEqual(vc.resolve(), "FAIL_COMMAND_OUTPUT")
        self.assertEqual(len(vc), 3)


class NaNInfDetectionTest(unittest.TestCase):
    def test_has_nan_or_inf_positive(self):
        self.assertTrue(m.has_nan_or_inf([1.0, float("nan"), 3.0]))
        self.assertTrue(m.has_nan_or_inf([1.0, float("inf")]))
        self.assertTrue(m.has_nan_or_inf([float("-inf")]))

    def test_has_nan_or_inf_negative(self):
        self.assertFalse(m.has_nan_or_inf([1.0, 2.0, 3.0]))
        self.assertFalse(m.has_nan_or_inf([]))

    def test_strip_nan_inf(self):
        self.assertEqual(m.strip_nan_inf([1.0, float("nan"), 2.0, float("inf")]), [1.0, 2.0])
        self.assertEqual(m.strip_nan_inf([float("nan")]), [])


class LocalFrameDisplacementTest(unittest.TestCase):
    def test_wrap_angle_rad(self):
        self.assertAlmostEqual(m.wrap_angle_rad(3.0 * math.pi), math.pi)
        self.assertAlmostEqual(m.wrap_angle_rad(-math.pi), math.pi)
        self.assertAlmostEqual(m.wrap_angle_rad(0.0), 0.0)

    def test_quaternion_tilt_upright(self):
        self.assertAlmostEqual(m.quaternion_tilt_deg(0.0, 0.0, 0.0, 1.0), 0.0)

    def test_quaternion_tilt_90_deg(self):
        qx = math.sin(math.pi / 4.0)
        qw = math.cos(math.pi / 4.0)
        self.assertAlmostEqual(m.quaternion_tilt_deg(qx, 0.0, 0.0, qw), 90.0, places=6)

    def test_local_frame_displacement_forward(self):
        fwd, lat, yaw_chg = m.local_frame_displacement(0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
        self.assertAlmostEqual(fwd, 1.0)
        self.assertAlmostEqual(lat, 0.0, places=6)
        self.assertAlmostEqual(yaw_chg, 0.0)

    def test_local_frame_displacement_with_yaw(self):
        # Start facing +Y (yaw = pi/2), move 1m forward in world +X
        # In body frame: world+X corresponds to body +Y (left)
        fwd, lat, yaw_chg = m.local_frame_displacement(0.0, 0.0, math.pi / 2.0, 1.0, 0.0, math.pi / 2.0)
        self.assertAlmostEqual(fwd, 0.0, places=6)
        self.assertAlmostEqual(lat, -1.0, places=6)

    def test_world_to_initial_body(self):
        fwd, lat = m.world_to_initial_body(0.0, 1.0, math.pi / 2.0)
        self.assertAlmostEqual(fwd, 1.0)
        self.assertAlmostEqual(lat, 0.0, places=6)


class PortAllocationTest(unittest.TestCase):
    def test_allocate_returns_dict(self):
        ports = m.allocate_unique_ports()
        self.assertIn("ros_master", ports)
        self.assertIn("gazebo_master", ports)
        self.assertGreater(ports["ros_master"], 1024)
        self.assertGreater(ports["gazebo_master"], 1024)
        self.assertNotEqual(ports["ros_master"], ports["gazebo_master"])

    def test_allocate_with_start_port(self):
        ports = m.allocate_unique_ports(start_port=11500)
        self.assertIn("ros_master", ports)
        self.assertGreaterEqual(ports["ros_master"], 11500)


class ArtifactPathValidationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.existing = os.path.join(self.tmp, "existing.txt")
        with open(self.existing, "w") as f:
            f.write("test")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_empty_path(self):
        valid, reason = m.validate_artifact_path("")
        self.assertFalse(valid)
        self.assertEqual(reason, "EMPTY_PATH")

    def test_not_found(self):
        valid, reason = m.validate_artifact_path("/nonexistent/path.pt")
        self.assertFalse(valid)
        self.assertEqual(reason, "NOT_FOUND")

    def test_valid_file(self):
        valid, reason = m.validate_artifact_path(self.existing)
        self.assertTrue(valid)
        self.assertIsNone(reason)

    def test_directory_rejected(self):
        valid, reason = m.validate_artifact_path(self.tmp)
        self.assertFalse(valid)
        self.assertEqual(reason, "IS_DIRECTORY")

    def test_validate_artifact_dir(self):
        valid, reason = m.validate_artifact_dir(self.tmp)
        self.assertTrue(valid)
        self.assertIsNone(reason)

        valid, reason = m.validate_artifact_dir("/nonexistent")
        self.assertFalse(valid)


class PolicySHA256Test(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.policy = os.path.join(self.tmp, "policy.pt")
        with open(self.policy, "wb") as f:
            f.write(b"dummy policy weights\n")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_compute_sha256(self):
        h = m.compute_sha256(self.policy)
        self.assertEqual(len(h), 64)
        self.assertTrue(m.is_valid_sha256_hex(h))

    def test_validate_policy_sha256_match(self):
        h = m.compute_sha256(self.policy)
        valid, result = m.validate_policy_sha256(self.policy, h)
        self.assertTrue(valid)
        self.assertEqual(result, h)

    def test_validate_policy_sha256_mismatch(self):
        valid, result = m.validate_policy_sha256(self.policy, "a" * 64)
        self.assertFalse(valid)
        self.assertNotEqual(result, "a" * 64)

    def test_is_valid_sha256_hex(self):
        self.assertTrue(m.is_valid_sha256_hex("a" * 64))
        self.assertTrue(m.is_valid_sha256_hex("0" * 64))
        self.assertFalse(m.is_valid_sha256_hex("xyz"))
        self.assertFalse(m.is_valid_sha256_hex("g" * 64))


class ClockMasterFailureTest(unittest.TestCase):
    def test_no_clock_data(self):
        self.assertEqual(m.classify_clock_master_failure([], gazebo_sim_advancing=True), "FAIL_ROS_CLOCK_PUBLISH")

    def test_clock_sparse(self):
        rows = [{"sim_time": float(i)} for i in range(3)]
        self.assertEqual(m.classify_clock_master_failure(rows, min_clock_samples=10), "FAIL_ROS_CLOCK_PUBLISH")

    def test_clock_stall(self):
        rows = [{"sim_time": float(i)} for i in range(20)]
        rows.insert(10, {"sim_time": 20.0})  # 9s gap after row 9 sim_time=9
        result = m.classify_clock_master_failure(rows, gazebo_sim_advancing=True, max_clock_gap=2.0)
        self.assertEqual(result, "FAIL_ROS_CLOCK_PUBLISH")

    def test_clock_ok(self):
        rows = [{"sim_time": float(i) * 0.05} for i in range(100)]
        self.assertEqual(m.classify_clock_master_failure(rows), "OK")

    def test_runner_clock_subscription(self):
        self.assertEqual(
            m.classify_clock_master_failure([], gazebo_sim_advancing=True, runner_received_clock=False),
            "FAIL_RUNNER_CLOCK_SUBSCRIPTION",
        )

    def test_master_mismatch(self):
        self.assertEqual(m.classify_clock_master_failure([], master_uri_match=False), "FAIL_MASTER_MISMATCH")


if __name__ == "__main__":
    unittest.main()
