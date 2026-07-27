#!/usr/bin/env python3
"""Transform FAST-LIO Odometry poses into the navigation map frame."""

import copy

import rospy
import tf
from nav_msgs.msg import Odometry


def compose_pose(frame_translation, frame_rotation, position, orientation):
    """Compose T_target_source with an odometry T_source_body pose."""
    matrix = tf.transformations.quaternion_matrix(frame_rotation)
    x = (matrix[0][0] * position.x + matrix[0][1] * position.y +
         matrix[0][2] * position.z + frame_translation[0])
    y = (matrix[1][0] * position.x + matrix[1][1] * position.y +
         matrix[1][2] * position.z + frame_translation[1])
    z = (matrix[2][0] * position.x + matrix[2][1] * position.y +
         matrix[2][2] * position.z + frame_translation[2])
    pose_rotation = (orientation.x, orientation.y,
                     orientation.z, orientation.w)
    rotation = tf.transformations.quaternion_multiply(
        frame_rotation, pose_rotation)
    return (x, y, z), rotation


class OdometryToMap:
    def __init__(self):
        self._input = rospy.get_param("~input", "/Odometry")
        self._output = rospy.get_param("~output", "/navigation/state_estimation")
        self._target_frame = rospy.get_param("~target_frame", "map").lstrip("/")
        self._listener = tf.TransformListener()
        self._publisher = rospy.Publisher(self._output, Odometry, queue_size=5)
        rospy.Subscriber(self._input, Odometry, self._callback, queue_size=5)

    def _callback(self, message):
        source_frame = message.header.frame_id.lstrip("/")
        if not source_frame:
            rospy.logwarn_throttle(2.0, "OdometryToMap: input frame_id is empty")
            return

        if source_frame == self._target_frame:
            output = copy.deepcopy(message)
        else:
            try:
                translation, rotation = self._listener.lookupTransform(
                    self._target_frame, source_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as exc:
                rospy.logwarn_throttle(
                    2.0, "OdometryToMap: waiting for %s -> %s: %s",
                    source_frame, self._target_frame, exc)
                return

            output = copy.deepcopy(message)
            position, orientation = compose_pose(
                translation, rotation, message.pose.pose.position,
                message.pose.pose.orientation)
            output.pose.pose.position.x = position[0]
            output.pose.pose.position.y = position[1]
            output.pose.pose.position.z = position[2]
            output.pose.pose.orientation.x = orientation[0]
            output.pose.pose.orientation.y = orientation[1]
            output.pose.pose.orientation.z = orientation[2]
            output.pose.pose.orientation.w = orientation[3]

        output.header.frame_id = self._target_frame
        self._publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("navigation_odometry_to_map")
    OdometryToMap()
    rospy.spin()
