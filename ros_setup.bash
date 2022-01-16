#!/bin/bash
set -e

source "$ROS_ROOT/install/setup.bash"

echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_ROOT: $ROS_ROOT"

exec "$@"