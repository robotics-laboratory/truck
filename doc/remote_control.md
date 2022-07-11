# Remote control

## Bluetooth (Linux)
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

## Control
- Push `L1 & R1` buttons - switch to auto mode
- Push `CROSS` button - switch to remote mode
- Left stick - curvature (left/right)
- Right stick - velocity (forward/backward)

