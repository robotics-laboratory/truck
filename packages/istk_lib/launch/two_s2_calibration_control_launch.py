#!/usr/bin/env python3

import yaml

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


CONFIG_PATH = "/truck/packages/istk_lib/config/laser_transforms.yaml"


def load_initial_pose_from_yaml():
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    left_to_right = data["left_to_right"]
    translation = left_to_right.get("translation", {})
    rotation_rpy = left_to_right.get("rotation_rpy", {})
    return [
        float(translation.get("x", 0.0)),
        float(translation.get("y", 0.0)),
        float(translation.get("z", 0.0)),
        float(rotation_rpy.get("roll", 0.0)),
        float(rotation_rpy.get("pitch", 0.0)),
        float(rotation_rpy.get("yaw", 0.0)),
    ]


def generate_launch_description():
    initial_pose = load_initial_pose_from_yaml()

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
                        "config_path": CONFIG_PATH,
                        "publish_rate_hz": 30.0,
                    }
                ],
            ),
            Node(
                package='pointcloud_to_laserscan',
                executable='laserscan_to_pointcloud_node',
                name='left_scan_to_cloud',
                output='screen',
                remappings=[
                    ('scan_in', '/left/scan'),
                    ('cloud', '/calib/left_cloud'),
                ],
                parameters=[{'queue_size': 10}],
            ),
            Node(
                package='pointcloud_to_laserscan',
                executable='laserscan_to_pointcloud_node',
                name='right_scan_to_cloud',
                output='screen',
                remappings=[
                    ('scan_in', '/right/scan'),
                    ('cloud', '/calib/right_cloud'),
                ],
                parameters=[{'queue_size': 10}],
            ),
            Node(
                package='multi_lidar_calibration',
                executable='multi_lidar_calibration_icp_node',
                name='multi_lidar_calibration_icp',
                output='screen',
                remappings=[
                    ('~/input/source_pointcloud', '/calib/left_cloud'),
                    ('~/input/target_pointcloud', '/calib/right_cloud'),
                ],
                parameters=[{
                    'initial_pose': initial_pose,
                    'config_path': CONFIG_PATH,
                    'publish_tf': False,
                    'max_iteration': 100,
                    'transform_epsilon': 1e-9,
                    'max_coorespondence_distance': 0.05,
                    'euclidean_fitness_epsilon': 0.5,
                    'ransac_outlier_rejection_threshold': 1.1,
                }],
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
                                "angle_min": -3.14159,
                                "angle_max": 3.14159,
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
        ]
    )
