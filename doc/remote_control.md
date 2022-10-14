# Remote control

## Connecting bluetooth gamepad

Note: you should run the following commands on a host device, not in a docker container.  
Start bluetoothctl. Enable the agent and set it as default:
```
[bluetooth]# agent on
[bluetooth]# default-agent
```

Power on the Bluetooth controller, and set it as discoverable: 
```
[bluetooth]# power on
[bluetooth]# discoverable on
[bluetooth]# pairable on 
```

Switch controller to pairing mode:
- **Dual shock 4:** hold down `PS & share` buttons for a few seconds
-  **IPEGA 9083:** TODO

Scan for devices:
```
[bluetooth]# scan on
[bluetooth]# devices
```

Using controller MAC, make pairing...
```
[bluetooth]# pair MAC
[bluetooth]# trust MAC_ADDR
```


Make connection and quit.
```
[bluetooth]# connect MAC_ADDR
[bluetooth]# quit
```

So, job is done. For more info use `help`.

## Launching

Build required nodes and import ROS environment:
```
cd packages
colcon build --merge-install --packages-up-to truck
source install/setup.sh
```

Launch:
```
ros2 launch truck remote.yaml
```

Check logs and ensure all nodes are started successfully:
- `joy_node` — expect "Opened joystick: ...."
- `control_proxy_node` — expect "Model acquired" and "Mode: OFF" messages
- `hardware_node` — expect "Hardware node initialized"

If you see all of these messages, it's time to drive!

## Control

- (❌) Cross button — switch to OFF mode (stop immediately, disable motor)
- (⭕) Circle button — switch to REMOTE mode (motion controlled via gamepad)
- (🔺) Triangle button — switch to AUTO mode (motion controlled programmatically)
- Left stick, horizontal axis — controls steering (left-right)
- Right stick, vertical axis — controls velocity (forward/backward)
