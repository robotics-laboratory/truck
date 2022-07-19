# Nodes & Topics

## Topics

### Planner
- `/motion/trajectory` [[nav_msgs/Path]([nav_msgs/Path](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html))] - target motion trajectory
- `/motion/command` [[truck_interfecase/Control](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_interfaces/msg/Control.msg)] - target of trajectory follower

### Remote control
- `/joy/set_feedback` [[sensos_msgs/JoyFeedback](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JoyFeedback.html)] - joypad input topic (rumble, leds and etc) 
- `/joy` [[sensos_msgs/Joy](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html)] - joypad output topic

### Control
- `/control/command` [[truck_interfecase/Control](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_interfaces/msg/Control.msg)] - control target
- `/control/mode` [[truck_interfecase/ControlMode](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_interfaces/msg/ControlMode.msg)] - control proxy mode (auto or remote control)

## Nodes
- [Control proxy](../packages/control_proxy/readme.md)
