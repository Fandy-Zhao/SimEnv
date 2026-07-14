#!/usr/bin/env python3
"""
Publish ``map -> camera_init`` static TF to connect FAST-LIO2's global frame
into the Gazebo world TF tree.
"""

import rospy
import tf2_ros
import geometry_msgs.msg as gm
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("map_to_camera_init_bridge")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    rospy.loginfo("Waiting for first /Odometry from FAST-LIO2 ...")
    try:
        first_odom = rospy.wait_for_message("/Odometry", Odometry, timeout=30.0)
    except rospy.ROSException:
        rospy.logerr("Timed out waiting for /Odometry.  Is FAST-LIO2 running?")
        return

    init_stamp = first_odom.header.stamp
    rospy.loginfo("First Odometry at sim time %.3f s", init_stamp.to_sec())

    body_in_map = None
    deadline = rospy.Time.now() + rospy.Duration(15)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        try:
            body_in_map = tf_buffer.lookup_transform(
                "map", "imu_link", init_stamp, rospy.Duration(3))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logdebug("TF lookup retry: %s", exc)
            rate.sleep()

    if body_in_map is None:
        rospy.logerr("Could not look up map -> imu_link at t=%.3f. "
                     "Falling back to latest TF.", init_stamp.to_sec())
        try:
            body_in_map = tf_buffer.lookup_transform(
                "map", "imu_link", rospy.Time(0), rospy.Duration(5))
        except Exception as e:
            rospy.logerr("Fallback also failed: %s", e)
            return

    # Publish map -> camera_init as a direct copy of map -> imu_link.
    # No extra rotation is applied here — the LiDAR 45° tilt is already
    # handled correctly by FAST-LIO2's extrinsic_R / extrinsic_T.
    # Applying Ry(-45°) here would tilt the entire camera_init world frame,
    # causing incorrect Odometry body axes (X pointing downward).
    t = gm.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "camera_init"
    t.transform = body_in_map.transform

    broadcaster.sendTransform(t)
    rospy.loginfo("Published static TF: map -> camera_init  (direct copy of map -> imu_link)")
    rospy.loginfo("  translation: (%.3f, %.3f, %.3f)",
                  t.transform.translation.x,
                  t.transform.translation.y,
                  t.transform.translation.z)

    rospy.spin()


if __name__ == "__main__":
    main()
