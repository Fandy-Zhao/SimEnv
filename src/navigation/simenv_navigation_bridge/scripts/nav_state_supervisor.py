#!/usr/bin/env python3
"""Navigation State Supervisor — latched state owner for recovery after restart.

Maintains authority over three topics so that bridge / navigation sub-stack
restarts do not require manual re-publishing:

  /navigation/enabled       — enables / disables cmd_vel forwarding
  /navigation/start_exploring — enables / disables DSV exploration
  /fsm/state_cmd            — tracks last commanded FSM state

Safety semantics:
  - Default disabled on first start (enabled=false, exploring=false, fsm=2)
  - Only user-explicit publish changes the remembered state
  - State is persisted to ROS parameter server for true cross-restart recovery
  - Feedback guard: ignores self-published latched echoes
  - Periodic re-publish ensures late-joining subscribers see current state
  - On restart, recovers previous state from param server
  - FSM state is re-published ONLY after a short readiness delay
"""

import rospy
from std_msgs.msg import Bool, Int8

_PARAM_NS = "/nav_state_supervisor"


class NavStateSupervisor:
    def __init__(self):
        # Latched publishers.
        self._pub_enabled = rospy.Publisher(
            "/navigation/enabled", Bool, latch=True, queue_size=1)
        self._pub_exploring = rospy.Publisher(
            "/navigation/start_exploring", Bool, latch=True, queue_size=1)
        self._pub_fsm = rospy.Publisher(
            "/fsm/state_cmd", Int8, latch=True, queue_size=1)

        # Attempt to recover persisted state from parameter server.
        self._enabled = rospy.get_param(_PARAM_NS + "/enabled", False)
        self._exploring = rospy.get_param(_PARAM_NS + "/exploring", False)
        # FSM: default FixedStand (2).  Recover only if explicitly persisted.
        self._fsm_state = rospy.get_param(_PARAM_NS + "/fsm_state", 2)

        # Subscribers — listen on the SAME topics to learn user intent.
        # Feedback guard: ignore messages identical to our current state.
        rospy.Subscriber("/navigation/enabled", Bool, self._enabled_cb, queue_size=1)
        rospy.Subscriber("/navigation/start_exploring", Bool, self._exploring_cb, queue_size=1)
        rospy.Subscriber("/fsm/state_cmd", Int8, self._fsm_cb, queue_size=5)

        # Publish enabled / exploring immediately (safe — bridge won't forward
        # unless FSM also commands Trotting and cmd_vel is fresh).
        self._pub_enabled.publish(Bool(data=self._enabled))
        self._pub_exploring.publish(Bool(data=self._exploring))

        # For FSM, defer re-publish by a short interval so that restarting
        # navigation nodes have time to initialise and the bridge can apply
        # its safety gating (require_trotting_state_cmd, command_timeout, etc.).
        # If the recovered state is NOT Trotting, publish immediately.
        if self._fsm_state != 4:
            self._pub_fsm.publish(Int8(data=self._fsm_state))
        else:
            rospy.Timer(rospy.Duration(4.0), self._deferred_fsm_publish, oneshot=True)

        # Periodic re-publish.
        self._republish_timer = rospy.Timer(
            rospy.Duration(3.0), self._republish_cb)

        rospy.loginfo("NavStateSupervisor: latched ready "
                       "(enabled=%s, exploring=%s, fsm=%d)%s",
                       self._enabled, self._exploring, self._fsm_state,
                       " [fsm deferred]" if self._fsm_state == 4 else "")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _enabled_cb(self, msg):
        new_val = bool(msg.data)
        if new_val == self._enabled:
            return
        self._enabled = new_val
        self._pub_enabled.publish(Bool(data=self._enabled))
        rospy.set_param(_PARAM_NS + "/enabled", self._enabled)
        rospy.loginfo("NavStateSupervisor: /navigation/enabled <- %s", self._enabled)

    def _exploring_cb(self, msg):
        new_val = bool(msg.data)
        if new_val == self._exploring:
            return
        self._exploring = new_val
        self._pub_exploring.publish(Bool(data=self._exploring))
        rospy.set_param(_PARAM_NS + "/exploring", self._exploring)
        rospy.loginfo("NavStateSupervisor: /navigation/start_exploring <- %s",
                       self._exploring)

    def _fsm_cb(self, msg):
        new_val = int(msg.data)
        if new_val == self._fsm_state:
            return
        self._fsm_state = new_val
        self._pub_fsm.publish(Int8(data=self._fsm_state))
        rospy.set_param(_PARAM_NS + "/fsm_state", self._fsm_state)
        rospy.loginfo("NavStateSupervisor: /fsm/state_cmd <- %d", self._fsm_state)

    # ------------------------------------------------------------------
    # Deferred FSM publish (safety gate after restart)
    # ------------------------------------------------------------------

    def _deferred_fsm_publish(self, _event):
        self._pub_fsm.publish(Int8(data=self._fsm_state))
        rospy.loginfo("NavStateSupervisor: deferred /fsm/state_cmd <- %d",
                       self._fsm_state)

    # ------------------------------------------------------------------
    # Re-publish helpers
    # ------------------------------------------------------------------

    def _publish_all(self):
        self._pub_enabled.publish(Bool(data=self._enabled))
        self._pub_exploring.publish(Bool(data=self._exploring))
        self._pub_fsm.publish(Int8(data=self._fsm_state))

    def _republish_cb(self, _event):
        self._publish_all()

    # ------------------------------------------------------------------
    # Public diagnostics
    # ------------------------------------------------------------------

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
