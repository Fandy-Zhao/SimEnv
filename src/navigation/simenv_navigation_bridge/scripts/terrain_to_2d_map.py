#!/usr/bin/env python3
"""
Minimal 2D occupancy grid projector.

Subscribes to the terrain map point cloud and accumulates a 2D occupancy grid.
Publishes the grid as nav_msgs/OccupancyGrid for use by the exploration recorder.

Usage:
  rosrun simenv_navigation_bridge terrain_to_2d_map.py \
    _input_cloud:=/navigation/terrain_map \
    _output_grid:=/navigation/occupancy_grid_2d
"""

import math
import threading

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


class TerrainTo2DMap:
    def __init__(self):
        # Parameters
        self.input_topic = rospy.get_param("~input_cloud", "/navigation/terrain_map")
        self.output_topic = rospy.get_param("~output_grid",
                                              "/navigation/occupancy_grid_2d")
        self.resolution = float(rospy.get_param("~resolution", 0.1))
        self.width_m = float(rospy.get_param("~width_m", 44.0))
        self.height_m = float(rospy.get_param("~height_m", 40.0))
        self.center_x = float(rospy.get_param("~center_x", 0.0))
        self.center_y = float(rospy.get_param("~center_y", 0.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 2.0))
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.min_z = float(rospy.get_param("~min_z", -0.3))
        self.max_z = float(rospy.get_param("~max_z", 1.2))
        self.occupied_threshold = int(rospy.get_param("~occupied_threshold", 2))

        # Grid dimensions
        self.grid_width = int(self.width_m / self.resolution)
        self.grid_height = int(self.height_m / self.resolution)

        # Origin: center of grid at (center_x, center_y)
        self.origin_x = self.center_x - self.width_m / 2.0
        self.origin_y = self.center_y - self.height_m / 2.0

        # Occupancy grid: count per cell
        self._lock = threading.Lock()
        self._hit_count = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)
        self._obs_count = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)
        self._point_count = 0

        # Subscribers
        self._sub = rospy.Subscriber(self.input_topic, PointCloud2,
                                      self._cloud_cb, queue_size=5)

        # Publisher
        self._pub = rospy.Publisher(self.output_topic, OccupancyGrid,
                                      queue_size=1, latch=True)

        # Timer
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self._publish_grid)

        rospy.loginfo("[TerrainTo2DMap] Initialized: %dx%d grid @ %.3fm/px, "
                       "origin=(%.1f, %.1f)",
                       self.grid_width, self.grid_height, self.resolution,
                       self.origin_x, self.origin_y)

    def _cloud_cb(self, msg):
        """Accumulate terrain point cloud into occupancy grid."""
        try:
            points = pc2.read_points(msg, field_names=("x", "y", "z"),
                                      skip_nans=True)
            with self._lock:
                for pt in points:
                    x, y, z = pt[0], pt[1], pt[2]
                    if z < self.min_z or z > self.max_z:
                        continue
                    col = int((x - self.origin_x) / self.resolution)
                    row = int((y - self.origin_y) / self.resolution)
                    if 0 <= col < self.grid_width and 0 <= row < self.grid_height:
                        self._hit_count[row, col] += 1
                        self._point_count += 1
        except Exception as e:
            rospy.logerr_throttle(10, "[TerrainTo2DMap] Error processing cloud: %s", e)

    def _publish_grid(self, _event):
        """Generate and publish OccupancyGrid from accumulated hits."""
        with self._lock:
            hits = self._hit_count.copy()
            total_points = self._point_count

        # Convert hit counts to OccupancyGrid values:
        #   hits >= occupied_threshold -> 100 (occupied)
        #   hits > 0 but < threshold   -> 0 (free, was seen)
        #   hits == 0                   -> -1 (unknown)
        data = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
        occupied_mask = hits >= self.occupied_threshold
        seen_mask = (hits > 0) & (~occupied_mask)
        data[occupied_mask] = 100
        data[seen_mask] = 0

        # Flip rows for OccupancyGrid (row 0 is top)
        data = np.flipud(data)

        msg = OccupancyGrid()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        msg.info = MapMetaData()
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data.ravel().tolist()

        try:
            self._pub.publish(msg)
        except Exception as e:
            rospy.logerr_throttle(10, "[TerrainTo2DMap] Publish error: %s", e)


if __name__ == "__main__":
    rospy.init_node("terrain_to_2d_map")
    node = TerrainTo2DMap()
    rospy.spin()
