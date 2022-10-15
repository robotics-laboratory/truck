# Visualization

## Overview
The node publishes all visualization topics. Color encoding for truck shape:
  - `off` - red
  - `blue` - remote
  - `green` - green

## Parameters
- `model_config` - path to model yaml file (see model module)

## Topics
### Input
- `/control/mode` [[truck_interfecase/ControlMode](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_interfaces/msg/ControlMode.msg)] - current control mode
- `/control/command` [[truck_interfecase/Control](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_interfaces/msg/Control.msg)] - target of trajectory controller
- `/odom` [[truck_interfecase/Control](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_interfaces/msg/Control.msg)] - current localization in odometry world

### Output
- `/visualization/ego` [[visualization_msgs/Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html)] - visualization of current truck pose.
