#!/usr/bin/env python3

import copy
import math
import os
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import yaml
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RearSectorFilter(Node):
    def __init__(self):
        super().__init__("rear_sector_filter")

        self.declare_parameter("input_scan", "/scan")
        self.declare_parameter("output_scan", "/scan_filtered")
        self.declare_parameter("config_path", "")
        self.declare_parameter("control_prefix", "/filter")

        # 180 degrees = behind the lidar in LaserScan local frame.
        self.declare_parameter("center_angle_deg", 180.0)

        # Keep center_angle +/- width/2.
        self.declare_parameter("width_deg", 60.0)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 2.0)

        # If true, keep all except selected sector.
        self.declare_parameter("invert", False)

        self.input_scan = self.get_parameter("input_scan").value
        self.output_scan = self.get_parameter("output_scan").value
        self.config_path = self.get_parameter("config_path").value
        self.control_prefix = self.get_parameter("control_prefix").value.rstrip("/")

        self.current_config = self.load_initial_config()
        self.startup_config = copy.deepcopy(self.current_config)

        self.center_angle = math.radians(float(self.current_config["center_angle_deg"]))
        self.half_width = math.radians(float(self.current_config["width_deg"]) / 2.0)
        self.range_min_limit = float(self.current_config["range_min"])
        self.range_max_limit = float(self.current_config["range_max"])
        self.invert = bool(self.current_config["invert"])

        self.pub = self.create_publisher(
            LaserScan,
            self.output_scan,
            qos_profile_sensor_data,
        )
        self.status_pub = self.create_publisher(
            String, f"{self.control_prefix}/status", 10
        )
        self.current_pub = self.create_publisher(
            Vector3, f"{self.control_prefix}/current", 10
        )

        self.sub = self.create_subscription(
            LaserScan,
            self.input_scan,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Vector3,
            f"{self.control_prefix}/adjust",
            self.on_adjust,
            10,
        )
        self.create_subscription(
            Vector3,
            f"{self.control_prefix}/set",
            self.on_set,
            10,
        )
        self.create_subscription(Empty, f"{self.control_prefix}/save", self.on_save, 10)
        self.create_subscription(Empty, f"{self.control_prefix}/reload", self.on_reload, 10)
        self.create_subscription(Empty, f"{self.control_prefix}/reset", self.on_reset, 10)

        self.publish_status(
            f"RearSectorFilter started: {self.input_scan} -> {self.output_scan}, "
            f"center={self.current_config['center_angle_deg']:.1f} deg, "
            f"width={self.current_config['width_deg']:.1f} deg, "
            f"range_max={self.range_max_limit:.2f} m, invert={self.invert}"
        )

    def load_initial_config(self) -> Dict[str, Any]:
        config = {
            "center_angle_deg": float(self.get_parameter("center_angle_deg").value),
            "width_deg": float(self.get_parameter("width_deg").value),
            "range_min": float(self.get_parameter("range_min").value),
            "range_max": float(self.get_parameter("range_max").value),
            "invert": bool(self.get_parameter("invert").value),
        }

        if not self.config_path:
            return config
        if not os.path.exists(self.config_path):
            return config

        with open(self.config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        data = data.get("rear_sector_filter", {})
        for key in config:
            if key in data:
                config[key] = data[key]

        return config

    def save_config(self) -> None:
        if not self.config_path:
            return

        data = {"rear_sector_filter": copy.deepcopy(self.current_config)}
        with open(self.config_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)

    def apply_config(self, config: Dict[str, Any]) -> None:
        self.current_config = {
            "center_angle_deg": float(config["center_angle_deg"]),
            "width_deg": float(config["width_deg"]),
            "range_min": float(config["range_min"]),
            "range_max": float(config["range_max"]),
            "invert": bool(config["invert"]),
        }
        self.center_angle = math.radians(self.current_config["center_angle_deg"])
        self.half_width = math.radians(self.current_config["width_deg"] / 2.0)
        self.range_min_limit = self.current_config["range_min"]
        self.range_max_limit = self.current_config["range_max"]
        self.invert = self.current_config["invert"]
        self.publish_current()

    def publish_current(self) -> None:
        msg = Vector3()
        msg.x = float(self.current_config["center_angle_deg"])
        msg.y = float(self.current_config["width_deg"])
        msg.z = float(self.current_config["range_max"])
        self.current_pub.publish(msg)

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.publish_current()
        self.get_logger().info(text)

    def on_adjust(self, msg: Vector3) -> None:
        config = copy.deepcopy(self.current_config)
        config["center_angle_deg"] += float(msg.x)
        config["width_deg"] = max(1.0, config["width_deg"] + float(msg.y))
        config["range_max"] = max(config["range_min"], config["range_max"] + float(msg.z))
        self.apply_config(config)
        self.publish_status(
            f"Adjusted sector: d_center={msg.x:.1f} deg, d_width={msg.y:.1f} deg, "
            f"d_range={msg.z:.2f} m"
        )

    def on_set(self, msg: Vector3) -> None:
        config = copy.deepcopy(self.current_config)
        config["center_angle_deg"] = float(msg.x)
        config["width_deg"] = max(1.0, float(msg.y))
        config["range_max"] = max(config["range_min"], float(msg.z))
        self.apply_config(config)
        self.publish_status(
            f"Set sector: center={msg.x:.1f} deg, width={msg.y:.1f} deg, range={msg.z:.2f} m"
        )

    def on_save(self, _: Empty) -> None:
        self.save_config()
        self.publish_status(f"Saved {self.config_path}")

    def on_reload(self, _: Empty) -> None:
        self.apply_config(self.load_initial_config())
        self.publish_status(f"Reloaded {self.config_path}")

    def on_reset(self, _: Empty) -> None:
        self.apply_config(copy.deepcopy(self.startup_config))
        self.publish_status("Reset to startup sector")

    def scan_callback(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header

        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        out.ranges = list(msg.ranges)

        if msg.intensities:
            out.intensities = list(msg.intensities)

        angle = msg.angle_min

        for i, r in enumerate(out.ranges):
            delta = abs(normalize_angle(angle - self.center_angle))
            inside_sector = delta <= self.half_width
            inside_range = self.range_min_limit <= r <= self.range_max_limit

            should_remove = inside_sector
            if self.invert:
                should_remove = not should_remove

            if inside_range and should_remove:
                out.ranges[i] = float("inf")
                if out.intensities:
                    out.intensities[i] = 0.0

            angle += msg.angle_increment

        self.pub.publish(out)


def main():
    rclpy.init()
    node = RearSectorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
