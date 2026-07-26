#!/usr/bin/env python3
"""Navigation State Supervisor — persistent latched state owner.

Mirrors the output topics it publishes on — when external tools
(auto.sh, manual rostopic pub, etc.) publish directly to the output
topics (/navigation/enabled, /fsm/state_cmd), the supervisor learns
the state and latches it.  Dedicated /navigation/request_* topics
are also supported for explicit user control.

Output topics (latched — consumed by bridge, exploration, controller):
  /navigation/enabled               std_msgs/Bool
  /navigation/start_exploring       std_msgs/Bool
  /fsm/state_cmd                    std_msgs/Int8

Request topics (optional, for explicit user control):
  /navigation/request_enabled       std_msgs/Bool
  /navigation/request_exploring     std_msgs/Bool
  /navigation/request_fsm_state     std_msgs/Int8

Safety semantics:
  - Default disabled on first start (enabled=false, exploring=false, fsm=2)
  - Mirrors direct publishes to output topics — learns state from auto.sh
  - State persisted to ROS param server for true cross-restart recovery
  - Periodic re-publish (1 Hz) ensures late-joining subscribers always
    see current state — no reliance on rostopic pub one-shots
  - On restart, recovers previous state from param server
"""

import rospy
from std_msgs.msg import Bool, Int8

_PARAM_NS = "/nav_state_supervisor"

# FSM values
FSM_FIXED_STAND = 2
FSM_TROTTING = 4


class NavStateSupervisor:
    def __init__(self):
        # ── Output publishers (latched — every subscriber sees current state) ──
        self._pub_enabled = rospy.Publisher(
            "/navigation/enabled", Bool, latch=True, queue_size=1)
        self._pub_exploring = rospy.Publisher(
            "/navigation/start_exploring", Bool, latch=True, queue_size=1)
        self._pub_fsm = rospy.Publisher(
            "/fsm/state_cmd", Int8, latch=True, queue_size=1)

        # ── Recover persisted state from parameter server ──
        self._enabled = rospy.get_param(_PARAM_NS + "/enabled", False)
        self._exploring = rospy.get_param(_PARAM_NS + "/exploring", False)
        self._fsm_state = rospy.get_param(_PARAM_NS + "/fsm_state",
                                          FSM_FIXED_STAND)

        # ── Subscribers: dedicated REQUEST topics (no feedback loop) ──
        rospy.Subscriber("/navigation/request_enabled", Bool,
                         self._req_enabled_cb, queue_size=1)
        rospy.Subscriber("/navigation/request_exploring", Bool,
                         self._req_exploring_cb, queue_size=1)
        rospy.Subscriber("/navigation/request_fsm_state", Int8,
                         self._req_fsm_cb, queue_size=5)

        # ── Mirror subscribers: learn state from direct output-topic publishes ──
        # auto.sh publishes /navigation/enabled and /fsm/state_cmd directly
        # (one-shot rostopic pub).  By subscribing to the same output topics
        # we publish on, the supervisor learns correct state without needing
        # auto.sh to know about request topics.  Callbacks are idempotent
        # (skip when value unchanged) → no feedback loop.
        rospy.Subscriber("/navigation/enabled", Bool,
                         self._req_enabled_cb, queue_size=5)
        rospy.Subscriber("/fsm/state_cmd", Int8,
                         self._req_fsm_cb, queue_size=5)

        # ── Publish initial state immediately ──
        self._pub_enabled.publish(Bool(data=self._enabled))
        self._pub_exploring.publish(Bool(data=self._exploring))

        if self._fsm_state != FSM_TROTTING:
            self._pub_fsm.publish(Int8(data=self._fsm_state))
        else:
            # Defer Trotting re-publish — safety gate after restart.
            # Use a short delay (0.5 s) so that new subscribers (bridge)
            # receive the latched value promptly; the periodic re-publish
            # at 1 Hz covers the steady state.
            rospy.Timer(rospy.Duration(0.5),
                        self._deferred_fsm_publish, oneshot=True)

        # ── Periodic re-publish at 1 Hz — keeps subscriber connections alive ──
        self._republish_timer = rospy.Timer(
            rospy.Duration(1.0), self._republish_cb)

        rospy.loginfo("NavStateSupervisor: ready "
                       "(enabled=%s, exploring=%s, fsm=%d)%s",
                       self._enabled, self._exploring, self._fsm_state,
                       " [fsm deferred]" if self._fsm_state == FSM_TROTTING
                       else "")

    # ── Request callbacks ────────────────────────────────────────────────

    def _req_enabled_cb(self, msg):
        new_val = bool(msg.data)
        if new_val == self._enabled:
            return
        self._enabled = new_val
        self._pub_enabled.publish(Bool(data=self._enabled))
        rospy.set_param(_PARAM_NS + "/enabled", self._enabled)
        rospy.loginfo("NavStateSupervisor: request enabled <- %s",
                       self._enabled)

    def _req_exploring_cb(self, msg):
        new_val = bool(msg.data)
        if new_val == self._exploring:
            return
        self._exploring = new_val
        self._pub_exploring.publish(Bool(data=self._exploring))
        rospy.set_param(_PARAM_NS + "/exploring", self._exploring)
        rospy.loginfo("NavStateSupervisor: request exploring <- %s",
                       self._exploring)

    def _req_fsm_cb(self, msg):
        new_val = int(msg.data)
        if new_val == self._fsm_state:
            return
        self._fsm_state = new_val
        self._pub_fsm.publish(Int8(data=self._fsm_state))
        rospy.set_param(_PARAM_NS + "/fsm_state", self._fsm_state)
        rospy.loginfo("NavStateSupervisor: request fsm_state <- %d",
                       self._fsm_state)

    # ── Deferred FSM publish ─────────────────────────────────────────────

    def _deferred_fsm_publish(self, _event):
        self._pub_fsm.publish(Int8(data=self._fsm_state))
        rospy.loginfo("NavStateSupervisor: deferred /fsm/state_cmd <- %d",
                       self._fsm_state)

    # ── Periodic re-publish (keeps subscriber connections persistent) ─────

    def _publish_all(self):
        self._pub_enabled.publish(Bool(data=self._enabled))
        self._pub_exploring.publish(Bool(data=self._exploring))
        self._pub_fsm.publish(Int8(data=self._fsm_state))

    def _republish_cb(self, _event):
        self._publish_all()

    # ── Public diagnostics ────────────────────────────────────────────────

    @property
    def navigation_enabled(self):
        return self._enabled

    @property
    def exploration_enabled(self):
        return self._exploring

    @property
    def fsm_state(self):
        return self._fsm_state


if __name__ == "__main__":
    rospy.init_node("nav_state_supervisor")
    supervisor = NavStateSupervisor()
    rospy.spin()
