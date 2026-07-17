#!/usr/bin/env bash
# The checked-in experiment never assumes a particular sibling worktree. Set
# SIMENV_BINARY_DEVEL to a verified Torch-enabled devel space before running.
: "${SIMENV_BINARY_DEVEL:?set SIMENV_BINARY_DEVEL to the verified devel directory}"
source "$SIMENV_BINARY_DEVEL/setup.bash"
OLD_SOURCE="$(cd "$SIMENV_BINARY_DEVEL/.." && pwd)/src"
ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH//${OLD_SOURCE}:/}"
ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH//:${OLD_SOURCE}/}"
CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH//${SIMENV_BINARY_DEVEL}:/}"
CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH//:${SIMENV_BINARY_DEVEL}/}"
export ROS_PACKAGE_PATH CMAKE_PREFIX_PATH
