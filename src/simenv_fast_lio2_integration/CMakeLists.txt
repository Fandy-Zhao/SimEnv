cmake_minimum_required(VERSION 3.0.2)
project(simenv_fast_lio2_integration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/scan_to_pointcloud2.py
  scripts/pointcloud_fields_check.py
  scripts/map_to_camera_init_bridge.py
  scripts/check_fast_lio2_extrinsics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
