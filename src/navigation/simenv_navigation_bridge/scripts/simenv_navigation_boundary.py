#!/usr/bin/env python3
"""Publish a rectangular navigation boundary polygon and visualization marker."""

import os
import time

import rospy
from geometry_msgs.msg import Point, Point32, PolygonStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class SimEnvNavigationBoundary:
    def __init__(self):
        self._boundary_topic = rospy.get_param("~boundary_topic", "/navigation/boundary")
        self._marker_topic = rospy.get_param("~marker_topic", "/navigation/boundary_marker")
        self._publish_rate = float(rospy.get_param("~publish_rate", 2.0))
        self._frame_id = rospy.get_param("~frame_id", "map")

        width = float(rospy.get_param("~building_width",
                      float(os.environ.get("BUILDING_WIDTH", 20.0))))
        length = float(rospy.get_param("~building_length",
                       float(os.environ.get("BUILDING_LENGTH", 36.0))))
        cx = float(rospy.get_param("~center_x",
                   float(os.environ.get("ROBOT_X", 0.0))))
        cy = float(rospy.get_param("~center_y",
                   float(os.environ.get("ROBOT_Y", 0.0))))
        shrink = float(rospy.get_param("~shrink_margin", 0.4))

        hw = width / 2.0 - shrink
        hl = length / 2.0 - shrink

        if hw <= 0.0 or hl <= 0.0:
            raise rospy.ROSInitException(
                "Invalid navigation boundary: width/length must exceed twice shrink_margin")

        self._polygon = PolygonStamped()
        self._polygon.header.frame_id = self._frame_id
        corners = [
            (cx - hw, cy - hl),
            (cx + hw, cy - hl),
            (cx + hw, cy + hl),
            (cx - hw, cy + hl),
        ]
        for x, y in corners:
            self._polygon.polygon.points.append(Point32(x=x, y=y, z=0.0))

        self._marker = Marker()
        self._marker.header.frame_id = self._frame_id
        self._marker.ns = "navigation_boundary"
        self._marker.id = 0
        self._marker.type = Marker.LINE_STRIP
        self._marker.action = Marker.ADD
        self._marker.pose.orientation.w = 1.0
        self._marker.scale.x = 0.05
        c = ColorRGBA(0.0, 1.0, 0.0, 0.8)
        self._marker.color = c
        for x, y in corners:
            self._marker.points.append(Point(x=x, y=y, z=0.0))
        self._marker.points.append(self._marker.points[0])  # close loop

        self._poly_pub = rospy.Publisher(self._boundary_topic, PolygonStamped, queue_size=1, latch=True)
        self._marker_pub = rospy.Publisher(self._marker_topic, Marker, queue_size=1, latch=True)

    def spin(self):
        sleep_period = 1.0 / self._publish_rate if self._publish_rate > 0.0 else 0.5
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            self._polygon.header.stamp = stamp
            self._marker.header.stamp = stamp
            self._poly_pub.publish(self._polygon)
            self._marker_pub.publish(self._marker)
            time.sleep(sleep_period)


if __name__ == "__main__":
    rospy.init_node("simenv_navigation_boundary")
    node = SimEnvNavigationBoundary()
    node.spin()
