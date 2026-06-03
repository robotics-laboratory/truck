#!/usr/bin/env python3

import copy
import math
import os
from typing import Any, Dict

import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Vector3
from std_msgs.msg import Empty, String
from tf2_ros import TransformBroadcaster


DEFAULT_CONFIG_PATH = "/truck/packages/istk_lib/config/laser_transforms.yaml"


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


def euler_from_quaternion(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def load_yaml(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if data is None:
        raise ValueError("Config YAML is empty")

    if "left_to_right" not in data:
        raise ValueError("Config YAML must contain top-level key: left_to_right")

    tf = data["left_to_right"]
    tf.setdefault("parent_frame", "laser_left")
    tf.setdefault("child_frame", "laser_right")
    tf.setdefault("translation", {})
    tf.setdefault("rotation_rpy", {})
    data.setdefault("base_frame", "base_link")
    return data


def save_yaml(path: str, data: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(
            data,
            f,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )


class LaserTransformControl(Node):
    def __init__(self):
        super().__init__("laser_transform_control")

        self.declare_parameter("config_path", DEFAULT_CONFIG_PATH)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.config_path = self.get_parameter("config_path").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.broadcaster = TransformBroadcaster(self)
        self.current_data = load_yaml(self.config_path)
        self.startup_data = copy.deepcopy(self.current_data)

        self.current_pub = self.create_publisher(
            TransformStamped,
            "/calibration/current_transform",
            10,
        )
        self.status_pub = self.create_publisher(String, "/calibration/status", 10)

        self.create_subscription(
            TransformStamped,
            "/calibration/set_transform",
            self.on_set_transform,
            10,
        )
        self.create_subscription(
            Vector3,
            "/calibration/adjust_transform",
            self.on_adjust_transform,
            10,
        )
        self.create_subscription(Empty, "/calibration/save", self.on_save, 10)
        self.create_subscription(Empty, "/calibration/reload", self.on_reload, 10)
        self.create_subscription(Empty, "/calibration/reset", self.on_reset, 10)
        self.create_subscription(
            TransformStamped,
            "/calibration/icp_transform",
            self.on_icp_transform,
            10,
        )

        self.create_timer(1.0 / self.publish_rate_hz, self.publish)

        self.publish_status(f"Loaded {self.config_path}")

    def left_to_right(self) -> Dict[str, Any]:
        return self.current_data["left_to_right"]

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def get_translation(self):
        translation = self.left_to_right().setdefault("translation", {})
        return (
            float(translation.get("x", 0.0)),
            float(translation.get("y", 0.0)),
            float(translation.get("z", 0.0)),
        )

    def get_rotation_rpy(self):
        rotation = self.left_to_right().setdefault("rotation_rpy", {})
        return (
            float(rotation.get("roll", 0.0)),
            float(rotation.get("pitch", 0.0)),
            float(rotation.get("yaw", 0.0)),
        )

    def set_transform_values(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> None:
        tf = self.left_to_right()
        translation = tf.setdefault("translation", {})
        rotation = tf.setdefault("rotation_rpy", {})
        translation["x"] = float(x)
        translation["y"] = float(y)
        translation["z"] = float(z)
        rotation["roll"] = float(roll)
        rotation["pitch"] = float(pitch)
        rotation["yaw"] = float(yaw)

    def make_left_to_right_tf(self) -> TransformStamped:
        tf = self.left_to_right()
        x, y, z = self.get_translation()
        roll, pitch, yaw = self.get_rotation_rpy()
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = tf.get("parent_frame", "laser_left")
        msg.child_frame_id = tf.get("child_frame", "laser_right")
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    def make_base_to_left_tf(self) -> TransformStamped:
        tf = self.left_to_right()
        x, y, z = self.get_translation()

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.current_data.get("base_frame", "base_link")
        msg.child_frame_id = tf.get("parent_frame", "laser_left")
        msg.transform.translation.x = -0.5 * x
        msg.transform.translation.y = -0.5 * y
        msg.transform.translation.z = -0.5 * z
        msg.transform.rotation.w = 1.0
        return msg

    def publish(self) -> None:
        left_to_right = self.make_left_to_right_tf()
        self.broadcaster.sendTransform(
            [
                self.make_base_to_left_tf(),
                left_to_right,
            ]
        )
        self.current_pub.publish(left_to_right)

    def on_set_transform(self, msg: TransformStamped) -> None:
        self.apply_transform_msg(msg)
        self.publish_status("Set transform from /calibration/set_transform")

    def on_icp_transform(self, msg: TransformStamped) -> None:
        self.apply_transform_msg(msg)
        self.publish_status("Applied ICP transform live. Use /calibration/save to persist.")

    def apply_transform_msg(self, msg: TransformStamped) -> None:
        roll, pitch, yaw = euler_from_quaternion(
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        )
        self.set_transform_values(
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
            roll,
            pitch,
            yaw,
        )
        tf = self.left_to_right()
        if msg.header.frame_id:
            tf["parent_frame"] = msg.header.frame_id
        if msg.child_frame_id:
            tf["child_frame"] = msg.child_frame_id

    def on_adjust_transform(self, msg: Vector3) -> None:
        x, y, z = self.get_translation()
        roll, pitch, yaw = self.get_rotation_rpy()
        self.set_transform_values(x + msg.x, y + msg.y, z, roll, pitch, yaw + msg.z)
        self.publish_status(
            f"Adjusted transform: dx={msg.x:.6f}, dy={msg.y:.6f}, dyaw={msg.z:.6f}"
        )

    def on_save(self, _: Empty) -> None:
        save_yaml(self.config_path, self.current_data)
        self.publish_status(f"Saved {self.config_path}")

    def on_reload(self, _: Empty) -> None:
        self.current_data = load_yaml(self.config_path)
        self.publish_status(f"Reloaded {self.config_path}")

    def on_reset(self, _: Empty) -> None:
        self.current_data = copy.deepcopy(self.startup_data)
        self.publish_status("Reset to startup transform")


def main():
    rclpy.init()
    node = LaserTransformControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
