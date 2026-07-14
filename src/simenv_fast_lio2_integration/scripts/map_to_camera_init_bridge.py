#!/usr/bin/env python3
"""
Publish ``map → camera_init`` static TF to connect FAST-LIO2's global frame
into the Gazebo world TF tree.

FAST-LIO2 initialises ``camera_init`` at the LiDAR's world pose when the first
scan arrives.  This node looks up ``map → laser_livox`` via TF at startup and
broadcasts the same transform as a static TF ``map → camera_init``, thereby
uniting the two previously disconnected TF sub-trees.
"""

import rospy
import tf2_ros
import geometry_msgs.msg as gm


def main():
    rospy.init_node("map_to_camera_init_bridge")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Wait until the TF tree is populated (laser_livox is a static TF from URDF)
    rospy.loginfo("Waiting for TF map → laser_livox ...")
    rate = rospy.Rate(10)
    lidar_in_map = None
    deadline = rospy.Time.now() + rospy.Duration(30)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        try:
            lidar_in_map = tf_buffer.lookup_transform("map", "laser_livox",
                                                       rospy.Time(0), rospy.Duration(2))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rate.sleep()

    if lidar_in_map is None:
        rospy.logerr("Could not look up map → laser_livox after 30 s. "
                     "Is Gazebo running with the robot model?")
        return

    # camera_init ≡ LiDAR world pose at t=0 (first scan)
    t = gm.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "camera_init"
    t.transform = lidar_in_map.transform

    broadcaster.sendTransform(t)
    rospy.loginfo("Published static TF: map → camera_init")
    rospy.loginfo("  translation: (%.3f, %.3f, %.3f)",
                  t.transform.translation.x,
                  t.transform.translation.y,
                  t.transform.translation.z)

    rospy.spin()


if __name__ == "__main__":
    main()
