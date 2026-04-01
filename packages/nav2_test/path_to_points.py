#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PointStamped
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class PathToPointsNode(Node):
    def __init__(self):
        super().__init__("path_to_points_node")

        # QoS for subscriber (reliable, transient local for latched topics)
        # qos = QoSProfile(
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,
        #     depth=10
        # )

        # Subscriber to /plan
        self.subscription = self.create_subscription(
            Path, "/plan", self.path_callback, 10
        )
        
        self.published = False

        # Publisher to /clicked_point
        self.publisher = self.create_publisher(PointStamped, "/clicked_point", 10)

        # Client for /reset_path service (std_srvs/srv/Empty)
        self.reset_client = self.create_client(Empty, "/reset_path")

        # Wait for service to become available (optional, not blocking)
        # We'll check availability in the callback

        self.get_logger().info(
            "Node initialized. Will call /reset_path before publishing points."
        )

    def call_reset_service(self):
        """Call the /reset_path service synchronously with a timeout."""
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "Service /reset_path not available. Proceeding without reset."
            )
            return False

        request = Empty.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            self.get_logger().info("Successfully called /reset_path service.")
            return True
        else:
            self.get_logger().warn(
                f"Service call to /reset_path failed: {future.exception()}"
            )
            return False

    def path_callback(self, msg: Path):
        """Callback for new Path message."""
        
        if self.published:
            return
        
        self.published = True
        
        num_poses = len(msg.poses)
        self.get_logger().info(f"Received new path with {num_poses} poses.")

        if num_poses == 0:
            self.get_logger().warn("Path contains no poses. Nothing to publish.")
            return

        # Call reset service before publishing
        self.call_reset_service()

        # Iterate over all poses and publish each point
        for i, pose_stamped in enumerate(msg.poses):
            point = PointStamped()
            point.header.frame_id = "map"
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x = pose_stamped.pose.position.x
            point.point.y = pose_stamped.pose.position.y
            point.point.z = pose_stamped.pose.position.z

            self.publisher.publish(point)
            self.get_logger().info(
                f"Published point {i + 1}/{num_poses}: "
                f"({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PathToPointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
