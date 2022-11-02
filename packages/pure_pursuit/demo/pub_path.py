import rclpy
import math

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry

class PathPublisher(Node):
    def __init__(self):
        super().__init__('PathPublisher')
        self.publisher_ = self.create_publisher(Path, '/motion/path', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        path = Path()
        path.header.frame_id = 'odom_ekf'
        path.header.stamp = now

        for i in range(1500):
            x = i / 100
            pose = PoseStamped()
            pose.header.frame_id = 'odom_ekf'
            pose.header.stamp = now
            pose.pose.position.x = x
            pose.pose.position.y = math.sin(x)
            path.poses.append(pose)

        self.publisher_.publish(path)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()