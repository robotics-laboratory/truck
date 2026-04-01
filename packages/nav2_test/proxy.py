#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from truck_msgs.msg import Control


class ProxyNode(Node):
    def __init__(self):
        super().__init__("proxy_node")

        # Subscribers
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(
            PoseStamped, "/goal_pose_raw", self.goal_pose_raw_callback, 10
        )
        self.create_subscription(Path, "/plan", self.plan_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Control, "/motion/command", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.plan_pub = self.create_publisher(Path, "/plan_fix", 10)

        self.get_logger().info("Proxy node started")

    def cmd_vel_callback(self, msg: Twist):
        control = Control()

        control.header.stamp = self.get_clock().now().to_msg()
        control.header.frame_id = "base"

        linear = msg.linear.x
        angular = msg.angular.z

        control.velocity = linear
        control.curvature = angular / abs(linear) if abs(linear) > 1e-6 else 0.0

        self.cmd_pub.publish(control)

    def goal_pose_raw_callback(self, msg: PoseStamped):
        goal = PoseStamped()

        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = msg.header.frame_id
        goal.pose = msg.pose

        self.goal_pub.publish(goal)

    def plan_callback(self, msg: Path):
        fixed_path = Path()
        fixed_path.header = msg.header

        for pose in msg.poses:
            p = PoseStamped()
            p.header = pose.header
            p.header.frame_id = msg.header.frame_id
            p.pose = pose.pose
            fixed_path.poses.append(p)

        self.plan_pub.publish(fixed_path)


def main(args=None):
    rclpy.init(args=args)
    node = ProxyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
