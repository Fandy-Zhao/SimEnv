cmake_minimum_required(VERSION 3.0.2)
project(simenv_navigation_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmd_vel_bridge.py
  scripts/exploration_result_recorder.py
  scripts/nav_state_supervisor.py
  scripts/registered_cloud_to_terrain_map.py
  scripts/simenv_navigation_boundary.py
  scripts/terrain_to_2d_map.py
  scripts/waypoint_frame_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
