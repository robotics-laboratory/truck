import rclpy
from rclpy.node import Node

import scipy as sc
import numpy as np

from scipy.interpolate import BSpline
from scipy.interpolate import splprep

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_srvs.srv import Empty

delta = 0.03

def ceil(x):
    return int(np.ceil(x))

def dist(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return np.sqrt(dx*dx + dy*dy)

def trace(a, b):
    n = ceil(dist(a, b)/delta) + 1
    return list(zip(np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n)))

def spline(points):
    point_n = len(points)

    if point_n < 2:
        return points

    if point_n == 2:
        return trace(points[0], points[1])

    subpose_n = 0
    for i in range(1, point_n):
        subpose_n += ceil(dist(points[i-1], points[i]) / delta) + 1

    data = [[x for x, y in points], [y for x, y in points]]

    tck, _ = splprep(data, s=0, k=2)
    t = np.linspace(0, 1, subpose_n)
    x = BSpline(tck[0], tck[1][0], tck[2])(t)
    y = BSpline(tck[0], tck[1][1], tck[2])(t)

    return list(zip(x, y))


class ControlDemo(Node):

    def __init__(self):
        super().__init__('control_demo')

        self.log = self.get_logger()

        self.last_odom = None
        self.points = []

        self.click_slot = self.create_subscription(
            PointStamped, '/clicked_point',
            self.on_click, 10) 

        self.odom_slot = self.create_subscription(
            Odometry, '/ekf/odometry/filtered',
            self.on_odom, 1)
        self.odom_slot

        self.path_timer = self.create_timer(0.2, self.publish_path)
        self.path_signal = self.create_publisher(Path, '/motion/path', 1)

        self.reset_srv = self.create_service(Empty, '/control_demo/reset_path', self.reset_path)

    def reset_path(self, request, response):
        self.get_logger().info('Reset path!')
        self.points = []
        return response

    def on_odom(self, odom):
        self.last_odom = odom

    def on_click(self, point_stamped):
        if not self.points and self.last_odom:
            point = (self.last_odom.pose.pose.position.x, self.last_odom.pose.pose.position.x)
            self.points.append(point)

        point = (point_stamped.point.x, point_stamped.point.y)
        self.points.append(point)

    def publish_path(self):
        now = self.get_clock().now().to_msg()

        path = Path()
        path.header.frame_id = 'odom_ekf'
        path.header.stamp = now

        for (x, y) in spline(self.points):
            pose = PoseStamped()
            pose.header.frame_id = 'odom_ekf'
            pose.header.stamp = now
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)

        self.path_signal.publish(path)


def main(args=None):
    rclpy.init(args=args)

    node = ControlDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()