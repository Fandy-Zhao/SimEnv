#!/usr/bin/env python3
"""
Publish ``map -> camera_init`` static TF to connect FAST-LIO2's global frame
into the Gazebo world TF tree.

FAST-LIO2 initialises ``camera_init`` at the LiDAR's world pose.  The Gazebo
LiDAR plugin outputs points in the sensor frame (laser_livox).  We look up
``map -> laser_livox`` and apply the pitch rotation that aligns the LiDAR
output convention with the Gazebo world frame.
"""

import math
import rospy
import tf2_ros
import geometry_msgs.msg as gm
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_multiply, quaternion_about_axis


def _rotate_by(tf_in, axis, angle_rad):
    """Right-multiply tf_in's rotation by angle_rad around axis."""
    q_extra = quaternion_about_axis(angle_rad, axis)
    q_in = [
        tf_in.rotation.x,
        tf_in.rotation.y,
        tf_in.rotation.z,
        tf_in.rotation.w,
    ]
    q_out = quaternion_multiply(q_in, q_extra)
    tf_out = gm.Transform()
    tf_out.translation = tf_in.translation
    tf_out.rotation.x = q_out[0]
    tf_out.rotation.y = q_out[1]
    tf_out.rotation.z = q_out[2]
    tf_out.rotation.w = q_out[3]
    return tf_out


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

    lidar_in_map = None
    deadline = rospy.Time.now() + rospy.Duration(15)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        try:
            lidar_in_map = tf_buffer.lookup_transform(
                "map", "laser_livox", init_stamp, rospy.Duration(3))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logdebug("TF lookup retry: %s", exc)
            rate.sleep()

    if lidar_in_map is None:
        rospy.logerr("Could not look up map -> laser_livox at t=%.3f. "
                     "Falling back to latest TF.", init_stamp.to_sec())
        try:
            lidar_in_map = tf_buffer.lookup_transform(
                "map", "laser_livox", rospy.Time(0), rospy.Duration(5))
        except Exception as e:
            rospy.logerr("Fallback also failed: %s", e)
            return

    # laser_livox is pitched down 45 deg (Ry(0.785)).  Rotate the transform
    # further so that camera_init aligns with the Gazebo world frame (Z-up).
    # The LiDAR plugin outputs points in its local frame -- the scan pattern
    # + the mounting pitch need to be undone for correct visualisation.
    # Un-tilt the LiDAR mounting pitch (45°) plus an additional 90° to align
    # the LiDAR's scan-output convention with the Gazebo world frame (Z-up).
    tf_aligned = _rotate_by(lidar_in_map.transform, (0, 1, 0), math.radians(135.0))

    t = gm.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "camera_init"
    t.transform = tf_aligned

    broadcaster.sendTransform(t)
    rospy.loginfo("Published static TF: map -> camera_init  (laser_livox + Ry(45 deg))")
    rospy.loginfo("  translation: (%.3f, %.3f, %.3f)",
                  t.transform.translation.x,
                  t.transform.translation.y,
                  t.transform.translation.z)

    rospy.spin()


if __name__ == "__main__":
    main()