#!/usr/bin/env python3
"""Publish controlled DSV inputs and check service/start/waypoint interfaces."""

import sys
import time

import rospy
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header
import sensor_msgs.point_cloud2 as pc2
import tf2_ros


def make_cloud(topic_frame, stamp):
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
    ]
    points = [(x, y, 0.0, 1.0) for x in (1.0, 2.0, 3.0) for y in (-1.0, 0.0, 1.0)]
    return pc2.create_cloud(Header(stamp=stamp, frame_id=topic_frame), fields, points)


def main():
    rospy.init_node("test_dsv_interface_smoke")
    timeout = float(rospy.get_param("~timeout", 20.0))
    waypoint_seen = {"value": False}

    odom_pub = rospy.Publisher("/navigation/state_estimation", Odometry, queue_size=1)
    terrain_pub = rospy.Publisher("/navigation/terrain_map", PointCloud2, queue_size=1)
    scan_pub = rospy.Publisher("/navigation/registered_scan", PointCloud2, queue_size=1)
    start_pub = rospy.Publisher("/navigation/start_exploring", Bool, queue_size=1, latch=True)
    stop_pub = rospy.Publisher("/navigation/stop_exploring", Bool, queue_size=1, latch=True)
    rospy.Subscriber(
        "/navigation/way_point",
        PointStamped,
        lambda msg: waypoint_seen.__setitem__("value", True),
        queue_size=1,
    )

    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_tf = TransformStamped()
    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id = "map"
    static_tf.child_frame_id = "laser_livox"
    static_tf.transform.rotation.w = 1.0
    tf_broadcaster.sendTransform(static_tf)

    try:
        rospy.wait_for_service("/navigation/drrtPlannerSrv", timeout=8.0)
    except rospy.ROSException:
        rospy.logerr("DSV planner service /navigation/drrtPlannerSrv was not available")
        return 2

    start_pub.publish(Bool(data=True))
    start = time.time()
    while not rospy.is_shutdown() and time.time() - start < timeout:
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "laser_livox"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        terrain_pub.publish(make_cloud("map", stamp))
        scan_pub.publish(make_cloud("laser_livox", stamp))
        if waypoint_seen["value"]:
            stop_pub.publish(Bool(data=True))
            return 0
        time.sleep(0.1)

    stop_pub.publish(Bool(data=True))
    rospy.logerr("DSV smoke failed: service exists but no /navigation/way_point was observed")
    return 1


if __name__ == "__main__":
    sys.exit(main())
