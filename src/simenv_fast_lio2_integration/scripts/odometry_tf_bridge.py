#!/usr/bin/env python3
"""Broadcast the frame transform represented by navigation Odometry."""
import math
import geometry_msgs.msg
import rospy
import tf2_ros
from nav_msgs.msg import Odometry


def finite_pose(pose):
    p, q = pose.position, pose.orientation
    return all(math.isfinite(v) for v in
               (p.x, p.y, p.z, q.x, q.y, q.z, q.w))


class OdometryTfBridge:
    def __init__(self):
        self.broadcaster = tf2_ros.TransformBroadcaster()
        topic = rospy.get_param("~odometry_topic", "/state_estimation")
        self.subscriber = rospy.Subscriber(topic, Odometry, self.callback,
                                           queue_size=50)

    def callback(self, msg):
        if not msg.header.frame_id or not msg.child_frame_id:
            rospy.logwarn_throttle(5, "Odometry TF bridge received empty frame")
            return
        if not finite_pose(msg.pose.pose):
            rospy.logwarn_throttle(5, "Odometry TF bridge rejected non-finite pose")
            return
        transform = geometry_msgs.msg.TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("odometry_tf_bridge")
    OdometryTfBridge()
    rospy.spin()
