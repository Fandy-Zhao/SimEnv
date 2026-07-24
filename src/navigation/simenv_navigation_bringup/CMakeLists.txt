cmake_minimum_required(VERSION 3.0.2)
project(simenv_navigation_bringup)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/test_falco_interface_smoke.py
  scripts/test_dsv_interface_smoke.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
