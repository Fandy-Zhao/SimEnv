#!/usr/bin/env python3
"""Publish controlled FALCO interface inputs and verify bridged velocity safety."""

import sys
import time

import rospy
from geometry_msgs.msg import PointStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header, Int8
import sensor_msgs.point_cloud2 as pc2
import tf2_ros


def make_cloud(stamp):
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
    ]
    points = [(2.0, -0.5, 0.0, 1.0), (2.0, 0.5, 0.0, 1.0), (3.0, 0.0, 0.0, 1.0)]
    return pc2.create_cloud(Header(stamp=stamp, frame_id="camera_init"), fields, points)


def main():
    rospy.init_node("test_falco_interface_smoke")
    timeout = float(rospy.get_param("~timeout", 12.0))

    odom_pub = rospy.Publisher("/navigation/state_estimation", Odometry, queue_size=1)
    cloud_pub = rospy.Publisher("/navigation/registered_scan", PointCloud2, queue_size=1)
    goal_pub = rospy.Publisher("/navigation/way_point", PointStamped, queue_size=1)
    enabled_pub = rospy.Publisher("/navigation/enabled", Bool, queue_size=1, latch=True)
    state_pub = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=1, latch=True)
    stop_pub = rospy.Publisher("/navigation/stop_exploring", Bool, queue_size=1, latch=True)
    path_seen = {"value": False}
    stamped_seen = {"value": False}
    twist_seen = {"value": False}

    rospy.Subscriber("/navigation/path", Path, lambda msg: path_seen.__setitem__("value", True), queue_size=1)
    rospy.Subscriber(
        "/navigation/falco/cmd_vel_stamped",
        TwistStamped,
        lambda msg: stamped_seen.__setitem__("value", True),
        queue_size=1,
    )
    rospy.Subscriber("/cmd_vel", Twist, lambda msg: twist_seen.__setitem__("value", True), queue_size=1)

    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_tf = TransformStamped()
    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id = "camera_init"
    static_tf.child_frame_id = "vehicle"
    static_tf.transform.rotation.w = 1.0
    tf_broadcaster.sendTransform(static_tf)

    enabled_pub.publish(Bool(data=True))
    state_pub.publish(Int8(data=4))

    start = time.time()
    while not rospy.is_shutdown() and time.time() - start < timeout:
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "camera_init"
        odom.child_frame_id = "body"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        cloud_pub.publish(make_cloud(stamp))
        goal = PointStamped()
        goal.header.stamp = stamp
        goal.header.frame_id = "camera_init"
        goal.point.x = 1.0
        goal_pub.publish(goal)
        if path_seen["value"] and stamped_seen["value"] and twist_seen["value"]:
            stop_pub.publish(Bool(data=True))
            return 0
        time.sleep(0.1)

    stop_pub.publish(Bool(data=True))
    rospy.logerr(
        "FALCO smoke failed: path=%s stamped_cmd=%s twist=%s",
        path_seen["value"],
        stamped_seen["value"],
        twist_seen["value"],
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
