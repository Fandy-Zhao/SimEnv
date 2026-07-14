#!/usr/bin/env python3
"""
Publish ``map → camera_init`` static TF to connect FAST-LIO2's global frame
into the Gazebo world TF tree.

FAST-LIO2 initialises ``camera_init`` at the body/IMU pose (upright, horizontal).
This node waits for the first ``/Odometry`` message, looks up ``map → imu_link``
at that exact timestamp, and broadcasts the resulting transform as a static TF
``map → camera_init``.

We use ``imu_link`` (not ``laser_livox``) because ``camera_init`` is aligned with
the body frame at init time, not the 45°-tilted LiDAR frame.
"""

import rospy
import tf2_ros
import geometry_msgs.msg as gm
from nav_msgs.msg import Odometry


def _flip_z(tf_in):
    """Compose tf_in with R_x(180°) to flip the Z axis.

    FAST-LIO2's camera_init Z points opposite to Gazebo's world Z.
    R_x(180°) preserves X while negating Y and Z, aligning the frames.
    """
    from tf.transformations import quaternion_multiply, quaternion_about_axis
    import math
    flip_q = quaternion_about_axis(math.pi, (1, 0, 0))  # R_x(180°)
    q_in = [
        tf_in.rotation.x,
        tf_in.rotation.y,
        tf_in.rotation.z,
        tf_in.rotation.w,
    ]
    q_out = quaternion_multiply(flip_q, q_in)
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

    # Wait for the first FAST-LIO2 Odometry to know *when* camera_init was fixed
    rospy.loginfo("Waiting for first /Odometry from FAST-LIO2 ...")
    try:
        first_odom = rospy.wait_for_message("/Odometry", Odometry, timeout=30.0)
    except rospy.ROSException:
        rospy.logerr("Timed out waiting for /Odometry.  Is FAST-LIO2 running?")
        return

    init_stamp = first_odom.header.stamp
    rospy.loginfo("First Odometry at sim time %.3f s", init_stamp.to_sec())

    # Look up map -> imu_link at that exact time (camera_init ≡ body at init)
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

    # Publish the static TF (with 180° Z flip to align FAST-LIO2 and Gazebo Z)
    t = gm.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "camera_init"
    t.transform = _flip_z(body_in_map.transform)

    broadcaster.sendTransform(t)
    rospy.loginfo("Published static TF: map -> camera_init")
    rospy.loginfo("  translation: (%.3f, %.3f, %.3f)  at sim-time %.3f s",
                  t.transform.translation.x,
                  t.transform.translation.y,
                  t.transform.translation.z,
                  init_stamp.to_sec())

    rospy.spin()


if __name__ == "__main__":
    main()
