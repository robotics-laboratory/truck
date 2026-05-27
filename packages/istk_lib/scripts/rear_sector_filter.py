#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RearSectorFilter(Node):
    def __init__(self):
        super().__init__("rear_sector_filter")

        self.declare_parameter("input_scan", "/scan")
        self.declare_parameter("output_scan", "/scan_filtered")

        # 180 degrees = behind the lidar in LaserScan local frame.
        self.declare_parameter("center_angle_deg", 180.0)

        # Keep center_angle +/- width/2.
        self.declare_parameter("width_deg", 60.0)

        # If true, keep all except selected sector.
        self.declare_parameter("invert", False)

        self.input_scan = self.get_parameter("input_scan").value
        self.output_scan = self.get_parameter("output_scan").value

        self.center_angle = math.radians(
            float(self.get_parameter("center_angle_deg").value)
        )
        self.half_width = math.radians(
            float(self.get_parameter("width_deg").value) / 2.0
        )

        self.invert = bool(self.get_parameter("invert").value)

        self.pub = self.create_publisher(
            LaserScan,
            self.output_scan,
            qos_profile_sensor_data,
        )

        self.sub = self.create_subscription(
            LaserScan,
            self.input_scan,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"RearSectorFilter started: "
            f"{self.input_scan} -> {self.output_scan}, "
            f"center={math.degrees(self.center_angle):.1f} deg, "
            f"width={math.degrees(self.half_width * 2.0):.1f} deg, "
            f"invert={self.invert}"
        )

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

            keep = inside_sector
            if self.invert:
                keep = not keep

            if not keep:
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
