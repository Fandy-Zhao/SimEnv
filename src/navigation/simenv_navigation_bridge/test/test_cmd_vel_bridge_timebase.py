#!/usr/bin/env python3
"""Unit tests for simulation-time command freshness."""

import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from cmd_vel_bridge import command_is_fresh  # noqa: E402


class CommandFreshnessTest(unittest.TestCase):
    def test_slow_wall_clock_does_not_expire_sim_fresh_command(self):
        self.assertTrue(command_is_fresh(10.0, 10.2, 0.5))

    def test_simulation_age_expires_command(self):
        self.assertFalse(command_is_fresh(10.0, 10.6, 0.5))

    def test_clock_reset_rejects_old_command(self):
        self.assertFalse(command_is_fresh(10.0, 2.0, 0.5))

    def test_uninitialised_stamp_is_not_fresh(self):
        self.assertFalse(command_is_fresh(0.0, 0.1, 0.5))


if __name__ == "__main__":
    unittest.main()
