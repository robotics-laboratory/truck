#!/usr/bin/env python3

import math
import os
from typing import Any, Dict

import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from ament_index_python.packages import get_package_share_directory


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_xy(x: float, y: float, yaw: float):
    c = math.cos(yaw)
    s = math.sin(yaw)

    return c * x - s * y, s * x + c * y


class DynamicTfFromYaml(Node):
    def __init__(self):
        super().__init__("dynamic_tf_from_yaml")

        default_config_path = os.path.join(
            get_package_share_directory("istk_lib"),
            "config",
            "laser_transforms.yaml",
        )

        self.declare_parameter("config_path", default_config_path)

        # Как часто перечитывать YAML.
        self.declare_parameter("reload_period_sec", 0.2)

        # Как часто публиковать TF.
        self.declare_parameter("publish_rate_hz", 30.0)

        self.config_path = self.get_parameter("config_path").value
        self.reload_period_sec = float(self.get_parameter("reload_period_sec").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.broadcaster = TransformBroadcaster(self)

        self.last_mtime = None
        self.transforms_params = None

        self.get_logger().info(f"Using config_path: {self.config_path}")
        self.get_logger().info("Publishing dynamic TF to /tf")

        self.load_config()

        self.reload_timer = self.create_timer(
            self.reload_period_sec,
            self.reload_if_changed,
        )

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_transforms,
        )

    def reload_if_changed(self):
        try:
            current_mtime = os.path.getmtime(self.config_path)
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found: {self.config_path}")
            return

        if self.last_mtime is None or current_mtime != self.last_mtime:
            self.get_logger().info("Config changed, reloading...")
            self.load_config()

    def load_yaml(self) -> Dict[str, Any]:
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config file not found: {self.config_path}")

        with open(self.config_path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file)

        if data is None:
            raise ValueError("Config YAML is empty")

        if "left_to_right" not in data:
            raise ValueError("Config YAML must contain top-level key: left_to_right")

        return data

    def load_config(self):
        try:
            data = self.load_yaml()

            base_frame = data.get("base_frame", "base")

            left_to_right = data["left_to_right"]
            left_frame = left_to_right.get("parent_frame", "laser_left")
            right_frame = left_to_right.get("child_frame", "laser_right")

            translation = left_to_right.get("translation", {})
            rotation_rpy = left_to_right.get("rotation_rpy", {})

            lr_x = float(translation.get("x", 0.0))
            lr_y = float(translation.get("y", 0.0))
            lr_z = float(translation.get("z", 0.0))

            lr_roll = float(rotation_rpy.get("roll", 0.0))
            lr_pitch = float(rotation_rpy.get("pitch", 0.0))
            lr_yaw = normalize_angle(float(rotation_rpy.get("yaw", 0.0)))

            # У двух почти противоположно направленных лидаров есть две
            # средние ориентации: вперед и назад. Если красная ось X у
            # base смотрит назад, поставь base_yaw_offset: pi в YAML.
            base_yaw_offset = normalize_angle(
                float(data.get("base_yaw_offset", 0.0))
            )

            # Хотим, чтобы ориентации лидаров в системе base были
            # симметричны относительно красной оси X base:
            #
            # yaw(base -> left)  = -lr_yaw / 2 + offset
            # yaw(base -> right) = +lr_yaw / 2 + offset
            #
            # Усредняем именно yaw, так как для навигации нужен поворот
            # вокруг вертикальной оси Z. Калиброванные roll и pitch между
            # лидарами сохраняются в transform left_to_right.
            base_to_left_roll = 0.0
            base_to_left_pitch = 0.0
            base_to_left_yaw = normalize_angle(
                -0.5 * lr_yaw + base_yaw_offset
            )

            # Ставим base ровно в середину между origins лидаров.
            #
            # Известно:
            #     t_lr — положение right в системе left.
            #     R_bl — поворот left в системе base.
            #
            # Тогда:
            #     t_bl = -0.5 * R_bl * t_lr
            #
            # Вращение делаем в плоскости XY, потому что base
            # усредняется по yaw.
            rotated_lr_x, rotated_lr_y = rotate_xy(
                lr_x,
                lr_y,
                base_to_left_yaw,
            )

            base_to_left_x = -0.5 * rotated_lr_x
            base_to_left_y = -0.5 * rotated_lr_y
            base_to_left_z = -0.5 * lr_z

            right_yaw_in_base = normalize_angle(
                base_to_left_yaw + lr_yaw
            )

            self.transforms_params = {
                "base_to_left": {
                    "parent": base_frame,
                    "child": left_frame,
                    "x": base_to_left_x,
                    "y": base_to_left_y,
                    "z": base_to_left_z,
                    "roll": base_to_left_roll,
                    "pitch": base_to_left_pitch,
                    "yaw": base_to_left_yaw,
                },
                "left_to_right": {
                    "parent": left_frame,
                    "child": right_frame,
                    "x": lr_x,
                    "y": lr_y,
                    "z": lr_z,
                    "roll": lr_roll,
                    "pitch": lr_pitch,
                    "yaw": lr_yaw,
                },
            }

            self.last_mtime = os.path.getmtime(self.config_path)

            self.get_logger().info(
                f"{base_frame} -> {left_frame}: "
                f"x={base_to_left_x:.6f}, "
                f"y={base_to_left_y:.6f}, "
                f"z={base_to_left_z:.6f}, "
                f"r={base_to_left_roll:.6f}, "
                f"p={base_to_left_pitch:.6f}, "
                f"yaw={base_to_left_yaw:.6f}"
            )

            self.get_logger().info(
                f"{left_frame} -> {right_frame}: "
                f"x={lr_x:.6f}, "
                f"y={lr_y:.6f}, "
                f"z={lr_z:.6f}, "
                f"r={lr_roll:.6f}, "
                f"p={lr_pitch:.6f}, "
                f"yaw={lr_yaw:.6f}"
            )

            self.get_logger().info(
                f"Headings in {base_frame}: "
                f"{left_frame}_yaw={base_to_left_yaw:.6f}, "
                f"{right_frame}_yaw={right_yaw_in_base:.6f}, "
                f"base_yaw_offset={base_yaw_offset:.6f}"
            )

        except Exception as error:
            self.get_logger().error(f"Failed to load config: {repr(error)}")

    def make_tf(self, params: Dict[str, Any]) -> TransformStamped:
        qx, qy, qz, qw = quaternion_from_euler(
            params["roll"],
            params["pitch"],
            params["yaw"],
        )

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = params["parent"]
        msg.child_frame_id = params["child"]

        msg.transform.translation.x = params["x"]
        msg.transform.translation.y = params["y"]
        msg.transform.translation.z = params["z"]

        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        return msg

    def publish_transforms(self):
        if self.transforms_params is None:
            return

        transforms = [
            self.make_tf(self.transforms_params["base_to_left"]),
            self.make_tf(self.transforms_params["left_to_right"]),
        ]

        self.broadcaster.sendTransform(transforms)


def main():
    rclpy.init()
    node = DynamicTfFromYaml()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
