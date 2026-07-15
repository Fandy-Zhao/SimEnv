#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
pointcloud_fields_check.py — Diagnostic script for PointCloud2 field inspection.

Subscribes to a PointCloud2 topic (default /scan_pointcloud2 or /livox/Pointcloud2)
and prints the field layout, checking for FAST-LIO2 compatibility.

Usage:
    rosrun simenv_fast_lio2_integration pointcloud_fields_check.py _topic:=/scan_pointcloud2
"""

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points


FIELD_NAMES = ["x", "y", "z", "intensity", "time", "timestamp", "offset_time", "ring", "line"]
RESULT_CACHE = {}


def callback(msg):
    if RESULT_CACHE:
        return  # Print once only

    rospy.loginfo("=== PointCloud2 Field Check ===")
    rospy.loginfo("header.frame_id: %s", msg.header.frame_id)
    rospy.loginfo("width: %d", msg.width)
    rospy.loginfo("height: %d", msg.height)
    rospy.loginfo("point_step: %d", msg.point_step)
    rospy.loginfo("row_step: %d", msg.row_step)
    rospy.loginfo("is_dense: %d", msg.is_dense)
    rospy.loginfo("--- fields (%d total) ---", len(msg.fields))

    field_found = {n: False for n in FIELD_NAMES}
    for f in msg.fields:
        rospy.loginfo("  name=%-16s offset=%-4d datatype=%-2d count=%-2d",
                      f.name, f.offset, f.datatype, f.count)
        if f.name in field_found:
            field_found[f.name] = True

    rospy.loginfo("--- FAST-LIO2 compatibility ---")
    for name in ["x", "y", "z"]:
        status = "OK" if field_found[name] else "MISSING"
        rospy.loginfo("  %-12s: %s", name, status)

    for name in ["intensity"]:
        status = "OK" if field_found[name] else "MISSING (optional for odometry, needed for reflectivity)"
        rospy.loginfo("  %-12s: %s", name, status)

    for name in ["time", "timestamp", "offset_time"]:
        status = "OK" if field_found[name] else "MISSING (per-point deskew disabled without this)"
        rospy.loginfo("  %-12s: %s", name, status)

    for name in ["ring", "line"]:
        status = "OK" if field_found[name] else "MISSING (scan line info not available)"
        rospy.loginfo("  %-12s: %s", name, status)

    rospy.loginfo("=== End PointCloud2 Field Check ===")
    RESULT_CACHE["done"] = True


def main():
    topic = rospy.get_param("~topic", "/scan_pointcloud2")
    rospy.init_node("pointcloud_fields_check", anonymous=True)
    sub = rospy.Subscriber(topic, PointCloud2, callback, queue_size=1)
    rospy.loginfo("Listening on %s ... (will print once and wait)", topic)
    rospy.spin()


if __name__ == "__main__":
    main()
