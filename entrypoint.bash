#!/bin/bash
set -e

service ssh start

echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_ROOT: $ROS_ROOT"

echo "CCX: $CCX"
echo "CCX_STANDART: $CCX_STANDART"

exec /bin/bash
