#!/bin/bash
set -e

# export ROS_DISTRO=galactic
# export ROS_ROOT=/opt/ros/${ROS_DISTRO}

source "$ROS_ROOT/setup.bash"

echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_ROOT: $ROS_ROOT"

exec /bin/bash