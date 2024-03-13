#!/bin/bash
. install/setup.bash

# ros2 topic pub /truck/target truck_interfaces/msg/Point "{x: 5, y: 3, theta: 1.57079632679489661923}" -1
# ros2 topic pub /truck/random_scene truck_interfaces/msg/RandomSeed "{seed: 1, probability: 0.01}" -1
# ros2 topic pub /truck/random_scene truck_interfaces/msg/RandomSeed "{seed: 1, probability: 0.0}" -1
ros2 topic pub /truck/control_command truck_interfaces/msg/Control "{curvature: 1.0, velocity: 1.0, acceleration: 1.0}" -1
