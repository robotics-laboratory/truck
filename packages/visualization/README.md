# Visualization

## Overview
The node publishes all visualization topics. Color encoding for control mode:
  - `off` - red
  - `blue` - remote
  - `green` - green

## Parameters
- `model_config` - path to model yaml file (see model module)

## Topics
### Input
- `/control/mode` [[truck_interfecase/ControlMode](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_msgs/msg/ControlMode.msg)] - control mode
- `/control/command` [[truck_interfecase/Control](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_msgs/msg/Control.msg)] - target of trajectory controller
- `/waypoints` [[truck_interfecase/Control](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_msgs/msg/Waypoints.msg)] - target waypoints to pass through
- `/ekf/odom/filtered` [[nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)] - localization in odometric world

### Output
- `/visualization/ego` [[visualization_msgs/Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html)] - pose in odometric world (color means control mode).
- `/visualization/arc`[[visualization_msgs/Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html)] - ccontrol command (curvature and velocity).
