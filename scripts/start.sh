#!/bin/bash
make -p log
cat /proc/$$/stat | awk '{print $5}' > log/group.pid

. install/setup.bash

ros2 run planning_node node >> log/planning_node.log 2>&1 &
disown -h $!
ros2 run unwrapping_node node >> log/unwrapping_node.log 2>&1 &
disown -h $!
ros2 launch rosbridge_server rosbridge_websocket_launch.xml >>log/rosbridge_server.log 2>&1 &
disown -h $!
