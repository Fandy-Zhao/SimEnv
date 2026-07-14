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

Usage:
    rosrun simenv_fast_lio2_integration scan_to_pointcloud2.py _scan_topic:=/scan
"""

import struct
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from std_msgs.msg import Header


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

        self.pub = rospy.Publisher(output_topic, PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber(scan_topic, PointCloud, self.callback, queue_size=10)

        rospy.loginfo("scan_to_pointcloud2: /scan -> %s (frame: laser_livox, xyzi32)",
                      output_topic)

    def callback(self, msg):
        header = msg.header
        header.frame_id = "laser_livox"
        points = [(p.x, p.y, p.z) for p in msg.points]
        cloud2 = _create_cloud_xyzi32(header, points)
        self.pub.publish(cloud2)


def main():
    rospy.init_node("scan_to_pointcloud2", anonymous=True)
    ScanToPointCloud2()
    rospy.spin()


if __name__ == "__main__":
    main()
