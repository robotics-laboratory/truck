import rclpy

from planning_interfaces.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from pure_pursuit_msgs.msg import Command

from math import sin, cos
from time import sleep
from random import random

rclpy.init()

node = rclpy.create_node("test")
path_pub = node.create_publisher(Path, "/truck/planned_path", 0)
odm_pub = node.create_publisher(Odometry, "/truck/current_state", 0)
path = Path()
yaw = 0.0
x = 0.0
y = 0.0
for i in range(20):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    path.path.poses.append(pose)
    yaw += (random() * 2 - 1) / 10
    x += sin(yaw)
    y += cos(yaw)

path_pub.publish(path)

odm = Odometry()

odm.pose.pose.orientation.x = 0.0
odm.pose.pose.orientation.y = 0.0
odm.pose.pose.orientation.z = 1.0
odm.pose.pose.orientation.w = 0.0

dt = 0.01

yaw = 0

def callback(cmd):
    global yaw
    print(cmd)
    odm.pose.pose.position.x += cmd.velocity.linear.x * dt
    odm.pose.pose.position.y += cmd.velocity.linear.y * dt
    yaw += cmd.velocity.angular.z * dt
    odm.pose.pose.orientation.w = cos(yaw / 2)
    odm_pub.publish(odm)
    sleep(0.1)

replay_slot = node.create_subscription(Command, "/truck/pure_pursuit_command", callback, 0)

odm_pub.publish(odm)
rclpy.spin(node)
