#!/bin/bash
set -e

echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_ROOT: $ROS_ROOT"

echo "CC/CXX: $CC/$CXX"
echo "CFLAGS: $CFLAGS"
echo "CXXFLAGS: $CXXFLAGS"

exec /bin/bash
