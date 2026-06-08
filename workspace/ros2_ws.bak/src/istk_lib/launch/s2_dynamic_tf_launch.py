#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("istk_lib"),
        "config",
        "laser_transforms.yaml",
    )

    return LaunchDescription([
        Node(
            package="istk_lib",
            executable="dynamic_tf_from_yaml.py",
            name="dynamic_tf_from_yaml",
            output="screen",
            parameters=[{
                "config_path": config_path,
                "reload_period_sec": 0.2,
                "publish_rate_hz": 30.0,
            }],
        ),
    ])
