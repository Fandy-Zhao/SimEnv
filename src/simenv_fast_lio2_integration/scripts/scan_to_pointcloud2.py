#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
scan_to_pointcloud2.py — Adapter node for FAST-LIO2

Subscribes to /scan (sensor_msgs/PointCloud) and republishes as
/scan_pointcloud2 (sensor_msgs/PointCloud2) in the laser_livox frame.
Each point carries x, y, z (FLOAT32) and intensity (FLOAT32).

The intensity field is required by FAST-LIO2's PointCloud2 path (lidar_type=4);
without it the feature extractor rejects every point as "not effective" and the
EKF diverges within seconds.

Optional rotation (``~rotation_yaw_deg``): rotates each point clockwise around
the Y axis by the given angle in degrees.  Default 90° (Ry(-90°)) so that the
point cloud aligns with the Odometry body frame convention.  Set to 0 to
disable.  Rotation is applied BEFORE the frame_id is stamped.

Usage:
    rosrun simenv_fast_lio2_integration scan_to_pointcloud2.py _scan_topic:=/scan
    rosrun simenv_fast_lio2_integration scan_to_pointcloud2.py _rotation_yaw_deg:=0
"""

import math
import struct
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from std_msgs.msg import Header


def _build_rotation_y(angle_deg):
    """Return a function that rotates (x, y, z) by ``angle_deg`` degrees around
    the Y axis.  Positive angle = counter-clockwise when viewed from +Y (ROS
    right-hand rule), so a NEGATIVE angle gives a clockwise rotation.

    Ry(theta):
         [ cos(theta),  0,  sin(theta)] [x]
         [ 0,          1,  0         ] [y]
         [ -sin(theta), 0,  cos(theta)] [z]
    """
    theta = math.radians(angle_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    def _rotate(x, y, z):
        return (
            cos_t * x + sin_t * z,
            y,
            -sin_t * x + cos_t * z,
        )
    return _rotate


def _create_cloud_xyzi32(header, points):
    """Build a PointCloud2 with fields x, y, z, intensity (all FLOAT32).

    ``points`` is a list of (x, y, z) tuples.  Intensity is set to 1.0 so
    that FAST-LIO2 curvature extraction is driven by geometry alone.
    """
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud = PointCloud2()
    cloud.header = header
    cloud.fields = fields
    cloud.point_step = 16
    cloud.row_step = cloud.point_step
    cloud.height = 1
    cloud.width = len(points)
    cloud.is_dense = True
    cloud.is_bigendian = False

    raw = bytearray()
    for px, py, pz in points:
        raw.extend(struct.pack("<ffff", px, py, pz, 1.0))
    cloud.data = bytes(raw)
    return cloud


class ScanToPointCloud2:
    def __init__(self):
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        output_topic = rospy.get_param("~output_topic", "/scan_pointcloud2")
        # Clockwise rotation around Y axis (deg).  Negative = clockwise.
        # Default -90° so the point cloud is rotated 90° clockwise around the
        # Odometry Y axis to align with the body-frame convention.
        rotation_yaw_deg = rospy.get_param("~rotation_yaw_deg", -90.0)

        self._rotate = None
        if abs(rotation_yaw_deg) > 1e-6:
            self._rotate = _build_rotation_y(rotation_yaw_deg)
            rospy.loginfo("scan_to_pointcloud2: Ry(%.1f deg) rotation enabled", rotation_yaw_deg)
        else:
            rospy.loginfo("scan_to_pointcloud2: rotation DISABLED (rotation_yaw_deg=0)")

        self.pub = rospy.Publisher(output_topic, PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber(scan_topic, PointCloud, self.callback, queue_size=10)

        rospy.loginfo("scan_to_pointcloud2: /scan -> %s (frame: laser_livox, xyzi32)",
                      output_topic)

    def callback(self, msg):
        header = msg.header
        header.frame_id = "laser_livox"
        if self._rotate is not None:
            points = [self._rotate(p.x, p.y, p.z) for p in msg.points]
        else:
            points = [(p.x, p.y, p.z) for p in msg.points]
        cloud2 = _create_cloud_xyzi32(header, points)
        self.pub.publish(cloud2)


def main():
    rospy.init_node("scan_to_pointcloud2", anonymous=True)
    ScanToPointCloud2()
    rospy.spin()


if __name__ == "__main__":
    main()
