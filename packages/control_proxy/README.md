# control_proxy

| [**docs**](../../doc/README.md) | [**packages**](../README.md) |
|---------------------------------|------------------------------|

## Overview
The node mixes remote and self-driving control modes:
- Joypad may be used as Red batton, mode switcher and remote control. More information about setting up and using you can find [here](../../doc/remote_control.md).
- If `mode` is `Auto`, node forwards control targets of planner
- If `mode` is `Remote`, you can drive truck with joypad
- Switching to `Remote` bmay be used as Red button
- Watchdog switch `mode`  to `Remote` if no commands from planner
- Watchdog switch `mode` to `Off` if lose joypad connection.

## Topics
### Input
- `/motion/command` [[truck_interfecase/Control](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_msgs/msg/Control.msg)] - target of trajectory controller
- `/joy` [[sensos_msgs/Joy](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html)] - joypad command topic

### Output
- `/joy/set_feedback` [[sensos_msgs/JoyFeedback](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JoyFeedback.html)] - joypad input topic for feedback (rumble, leds and etc) 
- `/control/command` [[truck_interfecase/Control](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_msgs/msg/Control.msg)] - control target command
- `/control/mode` [[truck_interfecase/ControlMode](https://github.com/robotics-laboratory/truck/blob/master/packages/truck_msgs/msg/ControlMode.msg)] - control proxy mode (`Auto`, `Remote`, `Off`)
