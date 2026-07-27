#!/usr/bin/env python3
"""Bridge FALCO TwistStamped commands into SimEnv's Trotting /cmd_vel."""

import math
import signal
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, Int8


def command_is_fresh(last_cmd_sim_time, now_sim_time, timeout):
    """Return whether a command is fresh on the ROS/simulation clock."""
    age = now_sim_time - last_cmd_sim_time
    return last_cmd_sim_time > 0.0 and 0.0 <= age <= timeout


class CmdVelBridge:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/navigation/falco/cmd_vel_stamped")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.enabled_topic = rospy.get_param("~enabled_topic", "/navigation/enabled")
        self.stop_topic = rospy.get_param("~stop_topic", "/navigation/stop_exploring")
        self.state_cmd_topic = rospy.get_param("~state_cmd_topic", "/fsm/state_cmd")

        self.max_linear_x = float(rospy.get_param("~max_linear_x", 0.20))
        self.max_linear_y = float(rospy.get_param("~max_linear_y", 0.00))
        self.max_angular_z = float(rospy.get_param("~max_angular_z", 0.30))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.5))
        self.enable_lateral_velocity = bool(rospy.get_param("~enable_lateral_velocity", False))
        self.require_navigation_enabled = bool(rospy.get_param("~require_navigation_enabled", True))
        self.require_trotting_state_cmd = bool(rospy.get_param("~require_trotting_state_cmd", True))
        self.publish_zero_when_disabled = bool(rospy.get_param("~publish_zero_when_disabled", True))
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self.trotting_state_value = int(rospy.get_param("~trotting_state_value", 4))

        self._lock = threading.Lock()
        self._last_cmd = None
        self._last_cmd_time = 0.0
        self._navigation_enabled = not self.require_navigation_enabled
        self._trotting_commanded = not self.require_trotting_state_cmd
        self._last_published_zero = False

        # Gate transition tracking for diagnostic logging.
        self._gate_was_open = self._gate_is_open()
        self._first_forward_logged = False
        self._last_rejection_log_time = 0.0
        self._rejection_log_interval = 5.0  # throttle rejection logs

        self._pub = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        rospy.Subscriber(self.input_topic, TwistStamped, self._cmd_cb, queue_size=1)
        rospy.Subscriber(self.enabled_topic, Bool, self._enabled_cb, queue_size=1)
        rospy.Subscriber(self.stop_topic, Bool, self._stop_cb, queue_size=1)
        rospy.Subscriber(self.state_cmd_topic, Int8, self._state_cmd_cb, queue_size=5)
        rospy.on_shutdown(self._publish_zero)

        rospy.loginfo("CmdVelBridge: initialised nav_enabled=%s trotting=%s "
                       "(require_nav=%s require_trot=%s trot_val=%d)",
                       self._navigation_enabled, self._trotting_commanded,
                       self.require_navigation_enabled, self.require_trotting_state_cmd,
                       self.trotting_state_value)

    # ── Unified gate check ────────────────────────────────────────────────

    def _gate_is_open(self):
        """Return True when all required safety conditions are satisfied."""
        nav_ok = (not self.require_navigation_enabled) or self._navigation_enabled
        fsm_ok = (not self.require_trotting_state_cmd) or self._trotting_commanded
        return nav_ok and fsm_ok

    def _log_gate_transition(self, now_open, reason=""):
        """Log gate open/close transitions once per edge."""
        if now_open and not self._gate_was_open:
            rospy.loginfo("CmdVelBridge: GATE OPENED (nav=%s, fsm_trot=%s) %s",
                           self._navigation_enabled, self._trotting_commanded,
                           reason)
        elif not now_open and self._gate_was_open:
            rospy.loginfo("CmdVelBridge: GATE CLOSED (nav=%s, fsm_trot=%s) %s",
                           self._navigation_enabled, self._trotting_commanded,
                           reason)
        self._gate_was_open = now_open

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _cmd_cb(self, msg):
        with self._lock:
            self._last_cmd = msg.twist
            self._last_cmd_time = rospy.Time.now().to_sec()

    def _enabled_cb(self, msg):
        prev = self._navigation_enabled
        with self._lock:
            self._navigation_enabled = bool(msg.data)
        if self._navigation_enabled != prev:
            rospy.loginfo("CmdVelBridge: /navigation/enabled <- %s",
                           self._navigation_enabled)
        if not msg.data and self.publish_zero_when_disabled:
            self._publish_zero()
        self._log_gate_transition(self._gate_is_open(),
                                   "enabled_cb(%s)" % self._navigation_enabled)

    def _stop_cb(self, msg):
        if msg.data:
            with self._lock:
                self._navigation_enabled = False if self.require_navigation_enabled else self._navigation_enabled
            if self.publish_zero_when_disabled:
                self._publish_zero()
            self._log_gate_transition(self._gate_is_open(), "stop_cb")

    def _state_cmd_cb(self, msg):
        prev = self._trotting_commanded
        with self._lock:
            self._trotting_commanded = msg.data == self.trotting_state_value
        if self._trotting_commanded != prev:
            rospy.loginfo("CmdVelBridge: /fsm/state_cmd <- %d (trotting=%s)",
                           msg.data, self._trotting_commanded)
        if msg.data != self.trotting_state_value and self.publish_zero_when_disabled:
            self._publish_zero()
        self._log_gate_transition(self._gate_is_open(),
                                   "state_cmd_cb(%d)" % msg.data)

    def _clamp(self, value, limit):
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))

    def _safe_twist(self, cmd):
        out = Twist()
        out.linear.x = self._clamp(cmd.linear.x, self.max_linear_x)
        if self.enable_lateral_velocity:
            out.linear.y = self._clamp(cmd.linear.y, self.max_linear_y)
        out.angular.z = self._clamp(cmd.angular.z, self.max_angular_z)
        return out

    def _publish_zero(self):
        try:
            self._pub.publish(Twist())
            self._last_published_zero = True
        except rospy.ROSException:
            pass

    def spin(self):
        sleep_period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.05
        while not rospy.is_shutdown():
            wall_now = time.monotonic()
            sim_now = rospy.Time.now().to_sec()
            with self._lock:
                cmd = self._last_cmd
                last_cmd_time = self._last_cmd_time
                enabled = self._navigation_enabled
                trotting = self._trotting_commanded

            fresh = cmd is not None and command_is_fresh(
                last_cmd_time, sim_now, self.command_timeout)
            gate_open = self._gate_is_open()
            allowed = gate_open and fresh

            if allowed:
                out = self._safe_twist(cmd)
                self._pub.publish(out)
                if not self._first_forward_logged:
                    self._first_forward_logged = True
                    rospy.loginfo("CmdVelBridge: first cmd_vel forwarded "
                                   "(lx=%.3f az=%.3f)",
                                   out.linear.x, out.angular.z)
                self._last_published_zero = False
            elif self.publish_zero_when_disabled and not self._last_published_zero:
                self._publish_zero()
                # Throttled rejection diagnostics.
                if (wall_now - self._last_rejection_log_time) >= self._rejection_log_interval:
                    self._last_rejection_log_time = wall_now
                    reasons = []
                    if not gate_open:
                        if self.require_navigation_enabled and not enabled:
                            reasons.append("nav_enabled=false")
                        if self.require_trotting_state_cmd and not trotting:
                            reasons.append("fsm_trot=false")
                    if not fresh:
                        reasons.append("cmd_stale(%.3fs sim)" % (sim_now - last_cmd_time)
                                       if cmd is not None else "no_cmd")
                    rospy.loginfo("CmdVelBridge: gate blocking — %s",
                                   ", ".join(reasons) if reasons else "unknown")

            time.sleep(sleep_period)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_bridge")
    bridge = CmdVelBridge()

    def _exit_handler(signum, _frame):
        bridge._publish_zero()
        rospy.signal_shutdown("received signal %s" % signum)
        sys.exit(0)

    signal.signal(signal.SIGINT, _exit_handler)
    signal.signal(signal.SIGTERM, _exit_handler)
    bridge.spin()
