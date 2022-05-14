#!/bin/bash
set -e

source "$ROS_ROOT/setup.bash"

echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_ROOT: $ROS_ROOT"

service ssh start

exec /bin/bash
