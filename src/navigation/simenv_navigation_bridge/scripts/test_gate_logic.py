#!/usr/bin/env python3
"""Unit tests for CmdVelBridge gate logic — no ROS master required.

Tests the safety gate decision logic in isolation.  The gate logic is
extracted from the CmdVelBridge class and tested against all required
cases from the specification.
"""

import sys
import threading
import time
import unittest


# ── Gate-under-test (extracted from cmd_vel_bridge.py) ──────────────────

class GateLogic:
    """Minimal gate logic mirroring CmdVelBridge without ROS deps."""

    def __init__(self,
                 require_navigation_enabled=True,
                 require_trotting_state_cmd=True,
                 trotting_state_value=4,
                 command_timeout=0.5):
        self.require_navigation_enabled = require_navigation_enabled
        self.require_trotting_state_cmd = require_trotting_state_cmd
        self.trotting_state_value = trotting_state_value
        self.command_timeout = command_timeout

        self._lock = threading.Lock()
        self._navigation_enabled = not self.require_navigation_enabled
        self._trotting_commanded = not self.require_trotting_state_cmd
        self._last_cmd = None
        self._last_cmd_time = 0.0

        # Transition tracking
        self._gate_was_open = self._gate_is_open()
        self.transitions = []  # (open_bool, reason_str)

    # ── Gate check (exact copy of cmd_vel_bridge._gate_is_open) ──────────

    def _gate_is_open(self):
        nav_ok = (not self.require_navigation_enabled) or self._navigation_enabled
        fsm_ok = (not self.require_trotting_state_cmd) or self._trotting_commanded
        return nav_ok and fsm_ok

    def gate_is_open(self):
        """Public accessor (same semantics as the private method)."""
        with self._lock:
            return self._gate_is_open()

    # ── Callbacks (mirror cmd_vel_bridge callbacks, minus ROS) ───────────

    def set_navigation_enabled(self, enabled: bool):
        with self._lock:
            prev = self._navigation_enabled
            self._navigation_enabled = enabled
        self._track_transition("nav_enabled=%s" % enabled)
        return prev != enabled

    def set_fsm_state(self, state: int):
        with self._lock:
            prev = self._trotting_commanded
            self._trotting_commanded = state == self.trotting_state_value
        self._track_transition("fsm=%d" % state)
        return prev != self._trotting_commanded

    def receive_cmd(self, twist_linear_x=0.0, twist_angular_z=0.0):
        with self._lock:
            self._last_cmd = (twist_linear_x, twist_angular_z)
            self._last_cmd_time = time.monotonic()

    def allow_command(self, now=None):
        """Return True if a command should be forwarded right now."""
        if now is None:
            now = time.monotonic()
        with self._lock:
            gate_open = self._gate_is_open()
            fresh = (self._last_cmd is not None
                     and (now - self._last_cmd_time) <= self.command_timeout)
        return gate_open and fresh

    def _track_transition(self, reason):
        now_open = self._gate_is_open()
        if now_open != self._gate_was_open:
            self.transitions.append((now_open, reason))
        self._gate_was_open = now_open

    @property
    def navigation_enabled(self):
        with self._lock:
            return self._navigation_enabled

    @property
    def trotting_commanded(self):
        with self._lock:
            return self._trotting_commanded


# ── Tests ────────────────────────────────────────────────────────────────

class TestGateInitialState(unittest.TestCase):
    """Case 1: gate starts closed with default settings."""

    def test_default_config_gate_starts_closed(self):
        g = GateLogic(require_navigation_enabled=True,
                      require_trotting_state_cmd=True)
        self.assertFalse(g.gate_is_open(),
                         "Gate must start closed when both requirements are on")
        self.assertFalse(g.navigation_enabled)
        self.assertFalse(g.trotting_commanded)

    def test_no_requirements_gate_starts_open(self):
        g = GateLogic(require_navigation_enabled=False,
                      require_trotting_state_cmd=False)
        self.assertTrue(g.gate_is_open(),
                        "Gate must start open when no requirements are set")


class TestGateConditions(unittest.TestCase):
    """Cases 2-4: gate opens only when both conditions hold."""

    def setUp(self):
        self.g = GateLogic()

    def test_nav_true_fsm_not_4_closed(self):
        """Case 2: nav=true, fsm!=4 → closed."""
        self.g.set_navigation_enabled(True)
        self.g.set_fsm_state(2)
        self.assertFalse(self.g.gate_is_open())

    def test_nav_false_fsm_4_closed(self):
        """Case 3: nav=false, fsm=4 → closed."""
        self.g.set_fsm_state(4)
        self.g.set_navigation_enabled(False)
        self.assertFalse(self.g.gate_is_open())

    def test_nav_true_fsm_4_open(self):
        """Case 4: nav=true, fsm=4 → open."""
        self.g.set_navigation_enabled(True)
        self.g.set_fsm_state(4)
        self.assertTrue(self.g.gate_is_open())

    def test_nav_true_fsm_6_closed(self):
        """FSM=6 (RL) is not Trotting (default trot_value=4)."""
        self.g.set_navigation_enabled(True)
        self.g.set_fsm_state(6)
        self.assertFalse(self.g.gate_is_open())


class TestGateClosing(unittest.TestCase):
    """Cases 5-6: gate closes immediately on state loss."""

    def setUp(self):
        self.g = GateLogic()
        self.g.set_navigation_enabled(True)
        self.g.set_fsm_state(4)
        self.assertTrue(self.g.gate_is_open(), "precondition: gate must be open")

    def test_nav_false_closes_gate_immediately(self):
        """Case 5: open → nav=false → immediately closed."""
        self.g.set_navigation_enabled(False)
        self.assertFalse(self.g.gate_is_open())
        self.assertGreaterEqual(len(self.g.transitions), 2,
                                "should have open + close transitions")

    def test_fsm_not_4_closes_gate_immediately(self):
        """Case 6: open → fsm=2 → immediately closed."""
        self.g.set_fsm_state(2)
        self.assertFalse(self.g.gate_is_open())

    def test_fsm_0_closes_gate(self):
        self.g.set_fsm_state(0)
        self.assertFalse(self.g.gate_is_open())


class TestLateJoiner(unittest.TestCase):
    """Cases 7-8: bridge starts late — still gets correct state."""

    def test_late_start_sees_existing_state(self):
        """Case 7: supervisor state already available when bridge starts."""
        # Simulate supervisor already running with nav=true, fsm=4.
        # A "late" bridge would receive latched messages from the supervisor.
        # We test that when callbacks fire with correct data, gate opens.
        g = GateLogic()
        self.assertFalse(g.gate_is_open(), "starts closed")
        g.set_navigation_enabled(True)
        g.set_fsm_state(4)
        self.assertTrue(g.gate_is_open(),
                        "gate must open after receiving both latched messages")

    def test_early_start_waits_for_state(self):
        """Case 8: bridge starts before supervisor publishes."""
        g = GateLogic()
        self.assertFalse(g.gate_is_open(), "starts closed")
        # Supervisor hasn't published yet — gate stays closed (safe).
        self.assertFalse(g.gate_is_open())
        # Later, supervisor publishes.
        g.set_navigation_enabled(True)
        g.set_fsm_state(4)
        self.assertTrue(g.gate_is_open(),
                        "gate must open after receiving belated state messages")

    def test_restart_recovery(self):
        """Case 9: bridge restart during exploration recovers state."""
        # A fresh bridge instance gets state via latched messages.
        g = GateLogic()
        self.assertFalse(g.gate_is_open(), "fresh start: gate closed")
        # Latched messages arrive.
        g.set_navigation_enabled(True)
        g.set_fsm_state(4)
        self.assertTrue(g.gate_is_open(),
                        "restarted bridge must recover state from latched messages")


class TestNoFalsePositive(unittest.TestCase):
    """Case 10: no valid state → must not open."""

    def test_no_state_no_open(self):
        g = GateLogic()
        self.assertFalse(g.gate_is_open())

    def test_only_nav_no_fsm_no_open(self):
        g = GateLogic()
        g.set_navigation_enabled(True)
        self.assertFalse(g.gate_is_open())

    def test_only_fsm_no_nav_no_open(self):
        g = GateLogic()
        g.set_fsm_state(4)
        self.assertFalse(g.gate_is_open())

    def test_unexpected_fsm_value_no_open(self):
        g = GateLogic()
        g.set_navigation_enabled(True)
        for bad_fsm in [-1, 0, 1, 3, 5, 7, 8, 100]:
            g.set_fsm_state(bad_fsm)
            self.assertFalse(g.gate_is_open(),
                             "gate must not open for fsm=%d" % bad_fsm)


class TestCommandForwarding(unittest.TestCase):
    """Test the full allow_command() flow including staleness."""

    def test_gate_closed_blocks_even_with_fresh_cmd(self):
        g = GateLogic()
        g.receive_cmd(0.5, 0.3)
        time.sleep(0.01)
        self.assertFalse(g.allow_command(),
                         "must block when gate is closed")

    def test_gate_open_cmd_stale_blocks(self):
        g = GateLogic()
        g.set_navigation_enabled(True)
        g.set_fsm_state(4)
        self.assertTrue(g.gate_is_open())
        # No command received — stale by definition.
        self.assertFalse(g.allow_command(),
                         "must block when no command has been received")

    def test_gate_open_fresh_cmd_allows(self):
        g = GateLogic(command_timeout=0.5)
        g.set_navigation_enabled(True)
        g.set_fsm_state(4)
        self.assertTrue(g.gate_is_open())
        g.receive_cmd(0.5, 0.3)
        time.sleep(0.01)
        self.assertTrue(g.allow_command(),
                        "must allow fresh command when gate is open")

    def test_command_stale_after_timeout(self):
        g = GateLogic(command_timeout=0.1)
        g.set_navigation_enabled(True)
        g.set_fsm_state(4)
        g.receive_cmd(0.5, 0.3)
        time.sleep(0.2)  # exceed timeout
        self.assertFalse(g.allow_command(),
                         "must block stale command")


class TestTransitionTracking(unittest.TestCase):
    """Verify transition tracking records gate edges correctly."""

    def test_tracks_open_and_close(self):
        g = GateLogic()
        self.assertEqual(g.transitions, [])
        # Open gate
        g.set_navigation_enabled(True)
        g.set_fsm_state(4)
        self.assertTrue(g.gate_is_open())
        self.assertGreaterEqual(len(g.transitions), 1)
        self.assertTrue(g.transitions[-1][0], "last transition should be open")

        # Close gate
        g.set_navigation_enabled(False)
        self.assertFalse(g.gate_is_open())
        self.assertGreaterEqual(len(g.transitions), 2)
        self.assertFalse(g.transitions[-1][0], "last transition should be close")

    def test_no_spurious_transitions(self):
        g = GateLogic()
        # Set FSM=4 twice — should not generate extra transitions.
        g.set_fsm_state(4)
        n_before = len(g.transitions)
        g.set_fsm_state(4)  # same value again
        self.assertEqual(len(g.transitions), n_before,
                         "re-setting same FSM value must not create a transition")

        # Set nav=true twice also.
        g.set_navigation_enabled(True)
        n_before = len(g.transitions)
        g.set_navigation_enabled(True)
        self.assertEqual(len(g.transitions), n_before,
                         "re-setting same nav value must not create a transition")


class TestRequireFlags(unittest.TestCase):
    """Test the require_* flags work correctly."""

    def test_no_nav_requirement_ignores_nav(self):
        g = GateLogic(require_navigation_enabled=False,
                      require_trotting_state_cmd=True)
        self.assertTrue(g.navigation_enabled,
                        "nav is true-by-default when require=False")
        g.set_fsm_state(4)
        self.assertTrue(g.gate_is_open(),
                        "gate must open with fsm=4 even when nav=false")

    def test_no_fsm_requirement_ignores_fsm(self):
        g = GateLogic(require_navigation_enabled=True,
                      require_trotting_state_cmd=False)
        self.assertTrue(g.trotting_commanded,
                        "fsm trot is true-by-default when require=False")
        g.set_navigation_enabled(True)
        self.assertTrue(g.gate_is_open(),
                        "gate must open with nav=true regardless of fsm")

    def test_custom_trotting_state_value(self):
        g = GateLogic(trotting_state_value=6)
        g.set_navigation_enabled(True)
        g.set_fsm_state(6)
        self.assertTrue(g.gate_is_open(),
                        "gate must open when fsm matches custom trot value")
        g.set_fsm_state(4)
        self.assertFalse(g.gate_is_open(),
                         "gate must close when fsm doesn't match custom trot value")


if __name__ == "__main__":
    unittest.main(verbosity=2)
