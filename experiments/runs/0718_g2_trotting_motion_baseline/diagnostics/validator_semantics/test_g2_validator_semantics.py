#!/usr/bin/env python3
"""Tests for G2-D1 fall validator frame semantics."""

from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g2_validator_semantics as validator


class ValidatorSemanticsTest(unittest.TestCase):
    def test_identity_has_zero_tilt(self):
        self.assertAlmostEqual(validator.body_tilt_deg(0.0, 0.0, 0.0, 1.0), 0.0)

    def test_normal_yaw_180_does_not_change_tilt(self):
        self.assertAlmostEqual(validator.body_tilt_deg(0.0, 0.0, 1.0, 0.0), 0.0, places=6)

    def test_roll_180_is_inverted_body_up(self):
        self.assertAlmostEqual(validator.body_tilt_deg(1.0, 0.0, 0.0, 0.0), 180.0, places=6)

    def test_inverse_quaternion_same_tilt_for_180_roll(self):
        self.assertAlmostEqual(validator.body_tilt_deg(-1.0, 0.0, 0.0, 0.0), 180.0, places=6)

    def test_quaternion_normalization(self):
        self.assertAlmostEqual(validator.body_tilt_deg(0.0, 0.0, 0.0, 2.0), 0.0)

    def test_normal_fixedstand_not_fall(self):
        sample = validator.PoseSample(1.0, 0.326, 0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        self.assertFalse(validator.legacy_height_fall([sample]))
        self.assertFalse(validator.semantic_fall([sample]))

    def test_real_side_fall_detected_by_tilt(self):
        sample = validator.PoseSample(1.0, 0.20, math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
        self.assertFalse(validator.legacy_height_fall([sample]))
        self.assertTrue(validator.semantic_fall([sample]))

    def test_low_body_height_detected(self):
        sample = validator.PoseSample(1.0, 0.079, 0.0, 0.0, 0.0, 1.0)
        self.assertTrue(validator.legacy_height_fall([sample]))
        self.assertTrue(validator.semantic_fall([sample]))

    def test_multiple_reasons_first_semantic_time(self):
        samples = [
            validator.PoseSample(1.0, 0.326, 0.0, 0.0, 0.0, 1.0),
            validator.PoseSample(2.0, 0.326, math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)),
            validator.PoseSample(3.0, 0.079, 0.0, 0.0, 0.0, 1.0),
        ]
        evidence = validator.summarize_fall(samples)
        self.assertEqual(evidence.first_semantic_fall_time, 2.0)
        self.assertEqual(evidence.first_legacy_fall_time, 3.0)


if __name__ == "__main__":
    unittest.main()
