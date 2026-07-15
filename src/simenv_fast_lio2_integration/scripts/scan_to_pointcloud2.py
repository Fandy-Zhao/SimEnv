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

Two configurable rotations (applied in order: Y first, then X):

  ``~rotation_y_deg`` (default -90): clockwise around Y axis.
      Ry(-90°):  (x, y, z) -> (-z, y, x)

  ``~rotation_x_deg`` (default 180): around X axis.
      Rx(180°):  (x, y, z) -> (x, -y, -z)

  Combined default: (x, y, z) -> (-z, -y, -x)

Set either to 0 to disable that axis.  Rotations are applied BEFORE the
frame_id is stamped.

Usage:
    rosrun simenv_fast_lio2_integration scan_to_pointcloud2.py _scan_topic:=/scan
    rosrun simenv_fast_lio2_integration scan_to_pointcloud2.py _rotation_y_deg:=0 _rotation_x_deg:=0
"""

import math
import struct
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from std_msgs.msg import Header


def _build_rotation(axis, angle_deg):
    """Return a function that rotates (x, y, z) by ``angle_deg`` degrees around
    ``axis`` ('x' or 'y').  Applies the right-hand rule (positive = CCW when
    viewed from the positive axis direction).
    """
    theta = math.radians(angle_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    if axis == 'y':
        def _rotate(x, y, z):
            return (
                cos_t * x + sin_t * z,
                y,
                -sin_t * x + cos_t * z,
            )
    elif axis == 'x':
        def _rotate(x, y, z):
            return (
                x,
                cos_t * y - sin_t * z,
                sin_t * y + cos_t * z,
            )
    else:
        raise ValueError("axis must be 'x' or 'y', got %r" % axis)
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
        # Y-axis rotation (deg).  Negative = clockwise when viewed from +Y.
        # Default -90°: (x,y,z)->(-z,y,x)
        rotation_y_deg = rospy.get_param("~rotation_y_deg", -90.0)
        # X-axis rotation (deg).  Positive = CCW when viewed from +X.
        # Default 180°: (x,y,z)->(x,-y,-z)
        rotation_x_deg = rospy.get_param("~rotation_x_deg", 180.0)

        # Build composed rotation: Rx ∘ Ry (Y first, then X)
        self._rotations = []
        if abs(rotation_y_deg) > 1e-6:
            self._rotations.append(_build_rotation('y', rotation_y_deg))
        if abs(rotation_x_deg) > 1e-6:
            self._rotations.append(_build_rotation('x', rotation_x_deg))

        if self._rotations:
            rospy.loginfo("scan_to_pointcloud2: Ry(%.1f°) ∘ Rx(%.1f°) enabled",
                          rotation_y_deg, rotation_x_deg)
        else:
            rospy.loginfo("scan_to_pointcloud2: all rotations DISABLED")

        self.pub = rospy.Publisher(output_topic, PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber(scan_topic, PointCloud, self.callback, queue_size=10)

        rospy.loginfo("scan_to_pointcloud2: /scan -> %s (frame: laser_livox, xyzi32)",
                      output_topic)

    def callback(self, msg):
        header = msg.header
        header.frame_id = "laser_livox"
        points = []
        for p in msg.points:
            x, y, z = p.x, p.y, p.z
            for rot in self._rotations:
                x, y, z = rot(x, y, z)
            points.append((x, y, z))
        cloud2 = _create_cloud_xyzi32(header, points)
        self.pub.publish(cloud2)


def main():
    rospy.init_node("scan_to_pointcloud2", anonymous=True)
    ScanToPointCloud2()
    rospy.spin()


if __name__ == "__main__":
    main()
