#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_left",
                namespace="left",
                output="screen",
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_56d944da3213ec119a9bfbef7a109228-if00-port0",
                        "serial_baudrate": 1000000,
                        "frame_id": "laser_left",
                        "inverted": True,
                        "angle_compensate": True,
                        "scan_mode": "DenseBoost",
                    }
                ],
            ),
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_right",
                namespace="right",
                output="screen",
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_8a17bc5aaf13ec11b6abf0ef7a109228-if00-port0",
                        "serial_baudrate": 1000000,
                        "frame_id": "laser_right",
                        "inverted": True,
                        "angle_compensate": True,
                        "scan_mode": "DenseBoost",
                    }
                ],
            ),
            # Replaces dynamic_tf_s2.py in this calibration launch.
            # Do not run dynamic_tf_s2.py at the same time: both publish laser_left -> laser_right.
            Node(
                package="istk_lib",
                executable="laser_transform_control.py",
                name="laser_transform_control",
                output="screen",
                parameters=[
                    {
                        "config_path": "/truck/packages/istk_lib/config/laser_transforms.yaml",
                        "publish_rate_hz": 30.0,
                    }
                ],
            ),
            ComposableNodeContainer(
                name="dual_laser_merger_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="dual_laser_merger",
                        plugin="merger_node::MergerNode",
                        name="dual_laser_merger",
                        parameters=[
                            {
                                "laser_1_topic": "/left/scan",
                                "laser_2_topic": "/right/scan",
                                "merged_scan_topic": "/merged/scan",
                                "merged_cloud_topic": "/merged/cloud",
                                "target_frame": "base_link",
                                "tolerance": 0.05,
                                "queue_size": 10,
                                "angle_min": -3.141592654,
                                "angle_max": 3.141592654,
                                "angle_increment": 0.002,
                                "scan_time": 0.1,
                                "range_min": 0.05,
                                "range_max": 30.0,
                                "min_height": -0.2,
                                "max_height": 0.2,
                                "inf_epsilon": 1.0,
                                "use_inf": False,
                                "enable_calibration": False,
                                "enable_shadow_filter": False,
                                "enable_average_filter": False,
                                "laser_1_x_offset": 0.0,
                                "laser_1_y_offset": 0.0,
                                "laser_1_yaw_offset": 0.0,
                                "laser_2_x_offset": 0.0,
                                "laser_2_y_offset": 0.0,
                                "laser_2_yaw_offset": 0.0,
                            }
                        ],
                    ),
                ],
            ),
            # If Foxglove bridge is not already running:
            # ros2 run foxglove_bridge foxglove_bridge
        ]
    )
