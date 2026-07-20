#!/usr/bin/env python3
"""Unit tests for G2-D1 pre-WAVE block reason classification."""

from __future__ import annotations

import io
import json
import os
import sys
import tempfile
import unittest

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import prewave_analyze as pa


class TestBlockReasonNames(unittest.TestCase):
    def test_cpp_reasons(self):
        self.assertEqual(pa.block_reason_name(0), "PRE_WAVE_BLOCK_NONE")
        self.assertEqual(pa.block_reason_name(101), "READINESS_HEIGHT_FALSE")
        self.assertEqual(pa.block_reason_name(102), "READINESS_STANCE_FALSE")
        self.assertEqual(pa.block_reason_name(103), "READINESS_CONTACT_FALSE")
        self.assertEqual(pa.block_reason_name(201), "NUMERICAL_GUARD_STATE")
        self.assertEqual(pa.block_reason_name(202), "NUMERICAL_GUARD_COMMAND")
        self.assertEqual(pa.block_reason_name(203), "NUMERICAL_GUARD_OUTPUT")
        self.assertEqual(pa.block_reason_name(301), "SAFETY_GUARD_ATTITUDE")
        self.assertEqual(pa.block_reason_name(302), "SAFETY_GUARD_CONTACT")

    def test_py_reasons(self):
        self.assertEqual(pa.block_reason_name(400), "FIXEDSTAND_UNSTABLE")
        self.assertEqual(pa.block_reason_name(401), "FALL_BEFORE_TROTTING")
        self.assertEqual(pa.block_reason_name(402), "FSM_TROTTING_NOT_ENTERED")
        self.assertEqual(pa.block_reason_name(406), "WAVE_START_NOT_REQUESTED")
        self.assertEqual(pa.block_reason_name(416), "PHYSICAL_FALL_BEFORE_WAVE")

    def test_unknown(self):
        self.assertEqual(pa.block_reason_name(999), "UNKNOWN_999")


class TestReadinessClassify(unittest.TestCase):
    def test_all_false(self):
        r = pa.classify_readiness(0)
        for k in r:
            self.assertFalse(r[k], f"flag {k} should be false for flags=0")

    def test_height_only(self):
        r = pa.classify_readiness(pa.FLAG_HEIGHT)
        self.assertTrue(r["height"])
        self.assertFalse(r["stance"])
        self.assertFalse(r["contact"])

    def test_hold(self):
        r = pa.classify_readiness(pa.FLAG_HOLD)
        self.assertTrue(r["hold"])
        self.assertFalse(r["met"])

    def test_all_flags(self):
        all_flags = 255  # all 8 bits
        r = pa.classify_readiness(all_flags)
        for k in r:
            self.assertTrue(r[k], f"flag {k} should be true for all_flags=255")

    def test_readiness_met(self):
        # height + stance + contact + linspeed + angspeed + tilt = flags 0-5
        flags = 1 | 2 | 4 | 8 | 16 | 32  # = 63, met=64, hold=128
        r = pa.classify_readiness(flags)
        self.assertTrue(r["height"])
        self.assertTrue(r["stance"])
        self.assertTrue(r["contact"])
        self.assertTrue(r["linspeed"])
        self.assertTrue(r["angspeed"])
        self.assertTrue(r["tilt"])
        self.assertFalse(r["met"])
        self.assertFalse(r["hold"])
        # met=64 means readiness conditions met
        r2 = pa.classify_readiness(flags | 64)
        self.assertTrue(r2["met"])
        self.assertFalse(r2["hold"])


class TestFirstBlockLatch(unittest.TestCase):
    """First block reason must be latched: first reason wins."""

    def test_latch_not_overwritten(self):
        """Once a block reason is set, it should not be overwritten."""
        s = pa.PreWaveTrialSummary()
        s.first_block_reason = 101
        s.first_block_reason_name = pa.block_reason_name(101)
        # A subsequent analysis should NOT overwrite 101 with 102
        self.assertEqual(s.first_block_reason, 101)
        self.assertEqual(s.first_block_reason_name, "READINESS_HEIGHT_FALSE")

    def test_reset_clears_latch(self):
        """A new PreWaveTrialSummary starts with reason=0, name cleared."""
        s = pa.PreWaveTrialSummary()
        self.assertEqual(s.first_block_reason, 0)
        self.assertEqual(s.first_block_reason_name, "")  # not yet classified


class TestTimelineClassification(unittest.TestCase):
    def test_timeline_e_fall_before_trotting(self):
        s = pa.PreWaveTrialSummary()
        s.trotting_entered = False
        s.fall_detected = True
        self.assertEqual(pa._classify_timeline(s), "E")

    def test_timeline_a_fall_before_readiness(self):
        s = pa.PreWaveTrialSummary()
        s.trotting_entered = True
        s.trotting_enter_sim_time_us = 1000
        s.fall_detected = True
        s.first_fall_sim_time_us = 2000
        s.hold_complete = False
        self.assertEqual(pa._classify_timeline(s), "A")

    def test_timeline_b_ready_but_no_wave(self):
        s = pa.PreWaveTrialSummary()
        s.trotting_entered = True
        s.hold_complete = True
        s.wave_all_entered = False
        s.numerical_guard = False
        self.assertEqual(pa._classify_timeline(s), "B")

    def test_timeline_d_wave_then_cancel(self):
        s = pa.PreWaveTrialSummary()
        s.trotting_entered = True
        s.hold_complete = True
        s.wave_all_entered = True
        s.wave_cancelled = True
        self.assertEqual(pa._classify_timeline(s), "D")

    def test_timeline_f_numerical_before_wave(self):
        s = pa.PreWaveTrialSummary()
        s.trotting_entered = True
        s.hold_complete = True
        s.wave_all_entered = False
        s.numerical_guard = True
        self.assertEqual(pa._classify_timeline(s), "F")


class TestFirstFailingCheckpoint(unittest.TestCase):
    def test_not_entered(self):
        s = pa.PreWaveTrialSummary()
        self.assertEqual(pa.determine_first_failing_checkpoint(s), "P1_FAIL_TROTTING_NOT_ENTERED")

    def test_fixedstand_unstable(self):
        s = pa.PreWaveTrialSummary()
        s.trotting_entered = True
        s.fixedstand_stable = False
        self.assertEqual(pa.determine_first_failing_checkpoint(s), "P0_FAIL_FIXEDSTAND_UNSTABLE")

    def test_height_false(self):
        s = pa.PreWaveTrialSummary()
        s.trotting_entered = True
        s.fixedstand_stable = True
        s.height_ready = False
        self.assertEqual(pa.determine_first_failing_checkpoint(s), "P3_FAIL_HEIGHT_READINESS")

    def test_wave_all_passed(self):
        s = pa.PreWaveTrialSummary()
        s.trotting_entered = True
        s.fixedstand_stable = True
        s.height_ready = True
        s.stance_ready = True
        s.contact_ready = True
        s.hold_complete = True
        s.wave_all_entered = True
        self.assertEqual(pa.determine_first_failing_checkpoint(s), "P10_PASS_WAVE_ALL_ENTERED")


class TestZeroCommandExpectedIdle(unittest.TestCase):
    """Zero-command vx=0: step not required, no WAVE_ALL is expected idle."""

    def _ready_summary(self):
        s = pa.PreWaveTrialSummary()
        s.trotting_entered = True
        s.fixedstand_stable = True
        s.height_ready = True
        s.stance_ready = True
        s.contact_ready = True
        s.hold_complete = True
        return s

    def test_zero_vx_no_wave_is_expected_idle(self):
        s = self._ready_summary()
        s.command_vx = 0.0
        s.step_required = False
        s.wave_all_entered = False
        self.assertEqual(
            pa.determine_first_failing_checkpoint(s),
            "P8_PASS_EXPECTED_NO_STEP_TRIGGER",
        )

    def test_nonzero_vx_no_wave_is_failure(self):
        s = self._ready_summary()
        s.command_vx = 0.5
        s.step_required = True
        s.wave_all_entered = False
        self.assertEqual(
            pa.determine_first_failing_checkpoint(s),
            "P8_FAIL_START_NOT_REQUESTED",
        )

    def test_threshold_boundary_below_is_idle(self):
        s = self._ready_summary()
        s.command_vx = 0.03
        s.step_required = abs(s.command_vx) > 0.03
        s.wave_all_entered = False
        self.assertEqual(
            pa.determine_first_failing_checkpoint(s),
            "P8_PASS_EXPECTED_NO_STEP_TRIGGER",
        )

    def test_threshold_boundary_above_is_failure(self):
        s = self._ready_summary()
        s.command_vx = 0.031
        s.step_required = abs(s.command_vx) > 0.03
        s.wave_all_entered = False
        self.assertEqual(
            pa.determine_first_failing_checkpoint(s),
            "P8_FAIL_START_NOT_REQUESTED",
        )

    def test_reason_code_registered(self):
        self.assertEqual(pa.block_reason_name(417), "EXPECTED_NO_STEP_TRIGGER")


class TestTrialClassification(unittest.TestCase):
    """Diagnostic trial validity: no WAVE_ALL is not automatically INVALID."""

    def test_no_wave_all_still_valid(self):
        s = pa.PreWaveTrialSummary()
        s.trotting_entered = True
        s.valid = True
        s.wave_all_entered = False
        # The trial is valid even without WAVE_ALL
        self.assertTrue(s.valid)
        self.assertFalse(s.wave_all_entered)

    def test_missing_fields_invalid(self):
        s = pa.PreWaveTrialSummary()
        s.invalid_reasons.append("CSV_READ_ERROR")
        self.assertFalse(s.valid)


class TestCrossTrialComparison(unittest.TestCase):
    def test_empty(self):
        result = pa.cross_trial_comparison([])
        self.assertEqual(result["num_trials"], 0)

    def test_single(self):
        s = pa.PreWaveTrialSummary()
        s.valid = True
        s.trotting_entered = True
        s.fixedstand_stable = True
        s.height_ready = True
        s.stance_ready = True
        s.contact_ready = True
        s.hold_complete = True
        s.wave_all_entered = True
        s.first_block_reason = 0
        s.first_block_reason_name = "PRE_WAVE_BLOCK_NONE"
        result = pa.cross_trial_comparison([s])
        self.assertEqual(result["num_trials"], 1)
        self.assertEqual(result["valid_trials"], 1)
        self.assertIn("P10_PASS_WAVE_ALL_ENTERED", result["first_checkpoint_counts"])


if __name__ == "__main__":
    unittest.main()
