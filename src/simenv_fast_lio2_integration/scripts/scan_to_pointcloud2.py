#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
scan_to_pointcloud2.py — Adapter node for FAST-LIO2

Subscribes to /scan (sensor_msgs/PointCloud) and republishes as
/scan_pointcloud2 (sensor_msgs/PointCloud2) in the laser_livox frame,
preserving only x, y, z fields.

This avoids the odom-frame transformation of the existing pointcloud2livox.py,
providing raw sensor-frame data suitable for SLAM.

Usage:
    rosrun simenv_fast_lio2_integration scan_to_pointcloud2.py _scan_topic:=/scan
"""

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud, PointCloud2


class ScanToPointCloud2:
    def __init__(self):
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        output_topic = rospy.get_param("~output_topic", "/scan_pointcloud2")

        self.pub = rospy.Publisher(output_topic, PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber(scan_topic, PointCloud, self.callback, queue_size=10)

        rospy.loginfo("scan_to_pointcloud2: /scan -> %s (frame: laser_livox)", output_topic)

    def callback(self, msg):
        header = msg.header
        header.frame_id = "laser_livox"
        points = [(p.x, p.y, p.z) for p in msg.points]
        cloud2 = pc2.create_cloud_xyz32(header, points)
        self.pub.publish(cloud2)


def main():
    rospy.init_node("scan_to_pointcloud2", anonymous=True)
    ScanToPointCloud2()
    rospy.spin()


if __name__ == "__main__":
    main()
