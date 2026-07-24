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
        self._last_cmd_time = rospy.Time(0)
        self._navigation_enabled = not self.require_navigation_enabled
        self._trotting_commanded = not self.require_trotting_state_cmd
        self._last_published_zero = False

        self._pub = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        rospy.Subscriber(self.input_topic, TwistStamped, self._cmd_cb, queue_size=1)
        rospy.Subscriber(self.enabled_topic, Bool, self._enabled_cb, queue_size=1)
        rospy.Subscriber(self.stop_topic, Bool, self._stop_cb, queue_size=1)
        rospy.Subscriber(self.state_cmd_topic, Int8, self._state_cmd_cb, queue_size=5)
        rospy.on_shutdown(self._publish_zero)

    def _cmd_cb(self, msg):
        with self._lock:
            self._last_cmd = msg.twist
            self._last_cmd_time = time.monotonic()

    def _enabled_cb(self, msg):
        with self._lock:
            self._navigation_enabled = bool(msg.data)
        if not msg.data and self.publish_zero_when_disabled:
            self._publish_zero()

    def _stop_cb(self, msg):
        if msg.data:
            with self._lock:
                self._navigation_enabled = False if self.require_navigation_enabled else self._navigation_enabled
            if self.publish_zero_when_disabled:
                self._publish_zero()

    def _state_cmd_cb(self, msg):
        with self._lock:
            self._trotting_commanded = msg.data == self.trotting_state_value
        if msg.data != self.trotting_state_value and self.publish_zero_when_disabled:
            self._publish_zero()

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
            now = time.monotonic()
            with self._lock:
                cmd = self._last_cmd
                last_cmd_time = self._last_cmd_time
                enabled = self._navigation_enabled
                trotting = self._trotting_commanded

            fresh = cmd is not None and (now - last_cmd_time) <= self.command_timeout
            allowed = enabled and trotting and fresh

            if allowed:
                self._pub.publish(self._safe_twist(cmd))
                self._last_published_zero = False
            elif self.publish_zero_when_disabled and not self._last_published_zero:
                self._publish_zero()

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
