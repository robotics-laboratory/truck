#!/usr/bin/env python3

import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from interface.srv import IsValid, Relocalize
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String


class InitialPoseToRelocalize(Node):
    def __init__(self):
        super().__init__("initialpose_to_relocalize")

        self.declare_parameter("pcd_path", "/truck/map.pcd")
        self.pcd_path = self.get_parameter("pcd_path").get_parameter_value().string_value

        self.declare_parameter("map_pcd_path", "/truck/map.pcd")
        self.map_pcd_path = (
            self.get_parameter("map_pcd_path").get_parameter_value().string_value
        )

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initialpose_callback, 10
        )

        self.relocalize_client = self.create_client(
            Relocalize, "/localizer/relocalize"
        )
        while not self.relocalize_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /localizer/relocalize service...")

        self.check_client = self.create_client(IsValid, "/localizer/relocalize_check")

        self.status_publisher = self.create_publisher(
            String, "/localization/status", 10
        )
        self.map_publisher = self.create_publisher(PointCloud2, "/localization/map", 1)

        self.timer_check = self.create_timer(1.0, self.timer_check_callback)
        self.timer_map = self.create_timer(10.0, self.publish_map)

        self.get_logger().info(
            "Node initialized. Ready to forward /initialpose, monitor status, and "
            "publish map."
        )

    def quaternion_to_euler(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw, pitch, roll

    def initialpose_callback(self, msg):
        pose = msg.pose.pose
        pos = pose.position
        ori = pose.orientation
        x, y, z = pos.x, pos.y, pos.z
        yaw, pitch, roll = self.quaternion_to_euler(ori)

        request = Relocalize.Request()
        request.pcd_path = self.pcd_path
        request.x = x
        request.y = y
        request.z = z
        request.yaw = yaw
        request.pitch = pitch
        request.roll = roll

        future = self.relocalize_client.call_async(request)
        future.add_done_callback(self.relocalize_response_callback)

        self.get_logger().info(
            f"Called relocalize: pcd_path={self.pcd_path}, "
            f"x={x:.3f}, y={y:.3f}, z={z:.3f}, "
            f"yaw={yaw:.3f}, pitch={pitch:.3f}, roll={roll:.3f}"
        )

    def relocalize_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Relocalize service call succeeded.")
        except Exception as error:
            self.get_logger().error(f"Relocalize service call failed: {error}")

    def timer_check_callback(self):
        if not self.check_client.service_is_ready():
            self.get_logger().debug(
                "Service /localizer/relocalize_check not ready yet."
            )
            return

        request = IsValid.Request()
        request.code = 0
        future = self.check_client.call_async(request)
        future.add_done_callback(self.check_response_callback)

    def check_response_callback(self, future):
        try:
            response = future.result()
            is_valid = response.valid

            status_msg = String()
            status_msg.data = f"Localization {'valid' if is_valid else 'invalid'}"
            self.status_publisher.publish(status_msg)
            self.get_logger().debug(f"Check service response: valid={is_valid}")
        except Exception as error:
            self.get_logger().error(f"Check service call failed: {error}")

            error_msg = String()
            error_msg.data = f"Check service error: {error}"
            self.status_publisher.publish(error_msg)

    def read_pcd_as_numpy(self, file_path):
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"PCD file not found: {file_path}")

        with open(file_path, "rb") as file:
            header = {}
            while True:
                line = file.readline().decode("utf-8").strip()
                if not line or line.startswith("#"):
                    continue

                parts = line.split(maxsplit=1)
                if len(parts) < 2:
                    break

                key, value = parts[0], parts[1]
                header[key] = value
                if key == "DATA":
                    break

            fields = header.get("FIELDS", "").split()
            sizes = list(map(int, header.get("SIZE", "").split()))
            types = header.get("TYPE", "").split()
            counts = list(map(int, header.get("COUNT", "").split()))
            width = int(header.get("WIDTH", 0))
            height = int(header.get("HEIGHT", 0))
            points = int(header.get("POINTS", width * height))
            data_format = header.get("DATA", "ascii").lower()

            try:
                idx_x = fields.index("x")
                idx_y = fields.index("y")
                idx_z = fields.index("z")
            except ValueError as error:
                raise ValueError("PCD file missing x, y, or z fields") from error

            if data_format == "ascii":
                points_list = []
                for _ in range(points):
                    line = file.readline().decode("utf-8").strip()
                    if not line:
                        break

                    values = line.split()
                    if len(values) < len(fields):
                        continue

                    try:
                        x = float(values[idx_x])
                        y = float(values[idx_y])
                        z = float(values[idx_z])
                        points_list.append([x, y, z])
                    except ValueError:
                        continue

                if not points_list:
                    raise ValueError("No points read from ASCII PCD file")

                return np.array(points_list, dtype=np.float32)

            if data_format == "binary":
                dtype_parts = []
                for name, size, data_type, count in zip(fields, sizes, types, counts):
                    if count != 1:
                        raise ValueError("Multi-count fields not supported yet")

                    if data_type == "F":
                        if size == 4:
                            np_type = np.float32
                        elif size == 8:
                            np_type = np.float64
                        else:
                            raise ValueError(f"Unsupported float size {size}")
                    elif data_type == "I":
                        if size == 1:
                            np_type = np.int8
                        elif size == 2:
                            np_type = np.int16
                        elif size == 4:
                            np_type = np.int32
                        elif size == 8:
                            np_type = np.int64
                        else:
                            raise ValueError(f"Unsupported int size {size}")
                    elif data_type == "U":
                        if size == 1:
                            np_type = np.uint8
                        elif size == 2:
                            np_type = np.uint16
                        elif size == 4:
                            np_type = np.uint32
                        elif size == 8:
                            np_type = np.uint64
                        else:
                            raise ValueError(f"Unsupported uint size {size}")
                    else:
                        raise ValueError(f"Unsupported type {data_type}")

                    dtype_parts.append((name, np_type))

                total_bytes = points * sum(sizes)
                data_bytes = file.read(total_bytes)
                if len(data_bytes) != total_bytes:
                    raise ValueError(
                        f"Expected {total_bytes} bytes, got {len(data_bytes)}"
                    )

                points_array = np.frombuffer(data_bytes, dtype=dtype_parts)
                x = points_array[fields[idx_x]]
                y = points_array[fields[idx_y]]
                z = points_array[fields[idx_z]]
                return np.column_stack((x, y, z)).astype(np.float32)

            raise ValueError(f"Unsupported data format: {data_format}")

    def publish_map(self):
        try:
            points = self.read_pcd_as_numpy(self.map_pcd_path)
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"
            msg = point_cloud2.create_cloud_xyz32(header, points)
            self.map_publisher.publish(msg)
            self.get_logger().info(
                f"Published map from {self.map_pcd_path} ({len(points)} points)"
            )
        except Exception as error:
            self.get_logger().error(f"Failed to read or publish map: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseToRelocalize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
