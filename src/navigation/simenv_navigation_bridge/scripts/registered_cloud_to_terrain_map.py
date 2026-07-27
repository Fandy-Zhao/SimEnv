#!/usr/bin/env python3
"""Filter registered point cloud into a local terrain map around the robot."""

import math
import time

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.point_cloud2 import read_points, create_cloud


def filter_terrain_points(points, odom, local_radius,
                          robot_self_filter_radius, min_relative_z,
                          max_relative_z, voxel_size):
    """Keep local floor and obstacle samples needed by DSV terrain analysis."""
    ox, oy, oz = odom[:3]
    inv_voxel = 1.0 / voxel_size
    r2_local = local_radius ** 2
    r2_self = robot_self_filter_radius ** 2
    voxel = {}

    for x, y, z in points:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        dx, dy, dz = x - ox, y - oy, z - oz
        d2 = dx * dx + dy * dy
        if d2 > r2_local or d2 < r2_self:
            continue
        if dz < min_relative_z or dz > max_relative_z:
            continue
        vk = (math.floor(x * inv_voxel), math.floor(y * inv_voxel),
              math.floor(z * inv_voxel))
        if vk not in voxel:
            voxel[vk] = (x, y, z)

    return list(voxel.values())


class RegisteredCloudToTerrainMap:
    def __init__(self):
        # --- topics ---
        self._input_cloud_topic = rospy.get_param("~input_cloud", "/navigation/registered_scan")
        self._state_topic = rospy.get_param("~state_estimation_topic", "/navigation/state_estimation")
        self._output_topic = rospy.get_param("~output_cloud", "/navigation/terrain_map")
        self._output_frame = rospy.get_param("~output_frame", "map")

        # --- filter parameters ---
        self._local_radius = float(rospy.get_param("~local_radius", 15.0))
        self._robot_self_filter_radius = float(rospy.get_param("~robot_self_filter_radius", 0.32))
        self._min_relative_z = float(rospy.get_param("~min_relative_z", -0.35))
        self._max_relative_z = float(rospy.get_param("~max_relative_z", 1.00))
        self._max_input_age = float(rospy.get_param("~max_input_age", 0.5))
        self._voxel_size = float(rospy.get_param("~voxel_size", 0.12))
        self._publish_rate = float(rospy.get_param("~publish_rate", 8.0))
        self._diag_throttle = float(rospy.get_param("~diagnostic_throttle_sec", 1.0))

        # --- state ---
        self._latest_odom = None  # (x, y, z, stamp)
        self._latest_cloud = None  # (header, points_array)

        self._pub = rospy.Publisher(self._output_topic, PointCloud2, queue_size=1)
        self._tf_listener = tf.TransformListener()
        rospy.Subscriber(self._input_cloud_topic, PointCloud2, self._cloud_cb, queue_size=1)
        rospy.Subscriber(self._state_topic, Odometry, self._odom_cb, queue_size=1)
        self._last_diag = 0.0

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        self._latest_odom = (pos.x, pos.y, pos.z, msg.header.stamp)

    def _cloud_cb(self, msg):
        pts = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self._latest_cloud = (msg.header, pts)

    def _transform_points(self, header, points):
        source_frame = header.frame_id.lstrip("/")
        target_frame = self._output_frame.lstrip("/")
        if not source_frame or source_frame == target_frame:
            return points

        stamp = header.stamp if header.stamp.to_sec() > 0.0 else rospy.Time(0)
        try:
            trans, rot = self._tf_listener.lookupTransform(target_frame, source_frame, stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(self._diag_throttle,
                                   "No TF for terrain map %s -> %s: %s", source_frame, target_frame, exc)
            return None

        matrix = tf.transformations.quaternion_matrix(rot)
        transformed = []
        for x, y, z in points:
            tx = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + trans[0]
            ty = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + trans[1]
            tz = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + trans[2]
            transformed.append((tx, ty, tz))
        return transformed

    def _filter(self, points, odom):
        return filter_terrain_points(
            points, odom, self._local_radius,
            self._robot_self_filter_radius, self._min_relative_z,
            self._max_relative_z, self._voxel_size)

    def spin(self):
        sleep_period = 1.0 / self._publish_rate if self._publish_rate > 0.0 else 0.125
        while not rospy.is_shutdown():
            now = time.time()
            odom = self._latest_odom
            cloud = self._latest_cloud

            if odom is None or cloud is None:
                if now - self._last_diag >= self._diag_throttle:
                    rospy.logwarn_throttle(self._diag_throttle, "Waiting for odometry or cloud data...")
                    self._last_diag = now
                time.sleep(sleep_period)
                continue

            cloud_age = (rospy.Time.now() - cloud[0].stamp).to_sec()
            odom_age = (rospy.Time.now() - odom[3]).to_sec()
            if cloud_age > self._max_input_age or odom_age > self._max_input_age:
                if now - self._last_diag >= self._diag_throttle:
                    rospy.logwarn_throttle(self._diag_throttle,
                                           "Stale data: cloud=%.2fs odom=%.2fs", cloud_age, odom_age)
                    self._last_diag = now
                time.sleep(sleep_period)
                continue

            map_points = self._transform_points(cloud[0], cloud[1])
            if map_points is None:
                time.sleep(sleep_period)
                continue

            pts = self._filter(map_points, odom)
            fields = [
                PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
            ]
            out_msg = create_cloud(cloud[0], fields, pts)
            out_msg.header.stamp = cloud[0].stamp
            out_msg.header.frame_id = self._output_frame
            self._pub.publish(out_msg)
            time.sleep(sleep_period)


if __name__ == "__main__":
    rospy.init_node("registered_cloud_to_terrain_map")
    node = RegisteredCloudToTerrainMap()
    node.spin()
