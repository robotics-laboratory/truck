#!/bin/bash
. install/setup.bash

# ros2 topic pub /random_scene planning_interfaces/msg/RandomSeed "{seed: 1, probability: 0.01}" -1
ros2 topic pub /random_scene planning_interfaces/msg/RandomSeed "{seed: 1, probability: 0.02}" -1
ros2 topic pub /target planning_interfaces/msg/Point "{x: 4, y: 3, theta: 90}" -1
