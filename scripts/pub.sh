#!/bin/bash
. install/setup.bash

ros2 topic pub /random_scene planning_interfaces/msg/RandomSeed "{seed: 0, probability: 0.3}" -1
ros2 topic pub /target planning_interfaces/msg/Point "{x: 4, y: 3, theta: 90}" -1