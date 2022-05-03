#!/bin/bash
set -e

echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_ROOT: $ROS_ROOT"

service ssh start

exec /bin/bash
