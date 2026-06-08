#!/usr/bin/env python3

import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_initial_pose_from_yaml():
    config_path = os.path.join(
        get_package_share_directory("istk_lib"),
        "config",
        "laser_transforms.yaml",
    )

    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    left_to_right = data["left_to_right"]

    translation = left_to_right.get("translation", {})
    rotation_rpy = left_to_right.get("rotation_rpy", {})

    x = float(translation.get("x", 0.0))
    y = float(translation.get("y", 0.0))
    z = float(translation.get("z", 0.0))

    roll = float(rotation_rpy.get("roll", 0.0))
    pitch = float(rotation_rpy.get("pitch", 0.0))
    yaw = float(rotation_rpy.get("yaw", 0.0))

    # multi_lidar_calibration expects:
    # [x, y, z, roll, pitch, yaw]
    return [x, y, z, roll, pitch, yaw]


def generate_launch_description():
    initial_pose = load_initial_pose_from_yaml()

    return LaunchDescription([
        # =========================
        # LEFT S2 lidar
        # =========================
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_left',
            namespace='left',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_56d944da3213ec119a9bfbef7a109228-if00-port0',
                'serial_baudrate': 1000000,
                'frame_id': 'laser_left',
                'inverted': True,
                'angle_compensate': True,
                'scan_mode': 'DenseBoost',
            }],
        ),

        # =========================
        # RIGHT S2 lidar
        # =========================
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_right',
            namespace='right',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_8a17bc5aaf13ec11b6abf0ef7a109228-if00-port0',
                'serial_baudrate': 1000000,
                'frame_id': 'laser_right',
                'inverted': True,
                'angle_compensate': True,
                'scan_mode': 'DenseBoost',
            }],
        ),

        # =========================
        # /left/scan -> /calib/left_cloud
        # =========================
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='left_scan_to_cloud',
            output='screen',
            remappings=[
                ('scan_in', '/left/scan'),
                ('cloud', '/calib/left_cloud'),
            ],
            parameters=[{
                'queue_size': 10,
            }],
        ),

        # =========================
        # /right/scan -> /calib/right_cloud
        # =========================
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='right_scan_to_cloud',
            output='screen',
            remappings=[
                ('scan_in', '/right/scan'),
                ('cloud', '/calib/right_cloud'),
            ],
            parameters=[{
                'queue_size': 10,
            }],
        ),

        # =========================
        # ICP calibration
        #
        # source = /calib/left_cloud
        # target = /calib/right_cloud
        #
        # initial_pose is loaded from:
        # istk_lib/config/laser_transforms.yaml
        #
        # YAML stores:
        # laser_left -> laser_right
        #
        # Expected format:
        # [x, y, z, roll, pitch, yaw]
        # =========================
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

                'max_iteration': 60,
                'transform_epsilon': 1e-9,

                # Since initial_pose is already close, keep ICP local.
                # If it says "Not enough correspondences", try 0.08 or 0.10.
                'max_coorespondence_distance': 0.05,

                'euclidean_fitness_epsilon': 1.0,
                'ransac_outlier_rejection_threshold': 0.5,
            }],
        ),
    ])
