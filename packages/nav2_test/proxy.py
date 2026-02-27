#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from truck_msgs.msg import Control


class ProxyNode(Node):
    def __init__(self):
        super().__init__('proxy_node')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.sub_goal_raw = self.create_subscription(
            PoseStamped,
            '/goal_pose_raw',
            self.goal_pose_raw_callback,
            10
        )
        self.sub_plan = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10
        )

        self.publisher = self.create_publisher(Control, '/motion/command', 10)

        goal_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 0)
        self.pub_plan_fix = self.create_publisher(Path, '/plan_fix', 0)

        self.get_logger().info('Proxy node started')

    def cmd_vel_callback(self, msg: Twist) -> None:
        control_msg = Control()
        control_msg.header = Header()
        control_msg.header.stamp = self.get_clock().now().to_msg()
        control_msg.header.frame_id = 'base'

        if abs(msg.angular.z) > 0.001 and abs(msg.linear.x) < 0.001:
            msg.linear.x = 0.1

        linear_x = msg.linear.x
        angular_z = msg.angular.z
        control_msg.velocity = linear_x

        if abs(linear_x) < 1e-6:
            control_msg.curvature = 0.0
        else:
            control_msg.curvature = angular_z / linear_x

        self.publisher.publish(control_msg)
        self.get_logger().debug(
            f'Published: velocity={linear_x:.3f}, curvature={control_msg.curvature:.3f}'
        )

    def goal_pose_raw_callback(self, msg: PoseStamped) -> None:
        goal_msg = PoseStamped()
        goal_msg.pose = msg.pose
        goal_msg.header = Header()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = msg.header.frame_id

        self.pub_goal.publish(goal_msg)
        self.get_logger().debug(
            f'Republished goal pose with stamp {goal_msg.header.stamp.sec}.{goal_msg.header.stamp.nanosec}'
        )

    def plan_callback(self, msg: Path) -> None:
        fixed_path = Path()
        fixed_path.header = msg.header

        for pose_stamped in msg.poses:
            new_pose = PoseStamped()
            new_pose.header = pose_stamped.header
            new_pose.header.frame_id = msg.header.frame_id
            new_pose.pose = pose_stamped.pose
            fixed_path.poses.append(new_pose)

        self.pub_plan_fix.publish(fixed_path)
        self.get_logger().debug(
            f'Republished /plan_fix with {len(fixed_path.poses)} poses, frame_id="{msg.header.frame_id}"'
        )


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


if __name__ == '__main__':
    main()
