cmake_minimum_required(VERSION 3.0.2)
project(simenv_navigation_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmd_vel_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
