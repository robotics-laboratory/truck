# Getting started
## Docker
We use docker for two purposes:
- Unified isolated environment for easy development and tests
- Dependency and code delivery system

All prebuild base images are stored at our registry ```cr.yandex/crp8hpfj5tuhlaodm4dl```. At this moment only ubuntu 20.04 is supported (images has some differences):
- **truck-jetson** (jetson runtime)
- **truck-amd64** (dev env, without cuda)

### Pull and run
```
# dev/amd64
docker-compose pull truck-amd64
docker-compose up -d truck-amd64

# jetson
docker-compose pull truck-jetson
docker-compose up -d truck-jetson

# just run another shell (preferred)
# exit doesn't stop the container
docker exec -it truck bash

# also you

# stop container
docker stop truck
```
### Build and push
If you need to build container after any changes, follow this steps. Up version in ```docker-compose.yaml``` if orresponding dockerfile is changed.

```
# dev/amd64
docker-compose build truck-amd64
docker-compose push truck-amd64

# jetson
docker-compose build truck-jetson
docker-compose push truck-jetson
```

## Dev
### Build
To build all packages run commands in `packages` dir.
```bash
colcon build --merge-install
```

To build your some single package.
```bash
colcon build --merge-install --packages-up-to package_name
```

Current installation dir is ```/truck/packages/install```. To run setup do
```bash
source install/setup.bash
```

### Test
All commands below are given for the `packages` dir.

To run tests:
```bash
colcon test --ctest-args tests --merge-install [package_selection_args]
```

Such a command will run tests of the specified package, as well as tests of all packages on which the specified package depends:
```bash
colcon test --ctest-args tests --merge-install --packages-up-to [package_name]
```

Such a command will run only the tests of the specified package:
```bash
colcon test --ctest-args tests --merge-install --packages-select [package_name]
```

## Pipeline
- `ros2 launch truck truck.yaml` - run full pipeline
- `ros2 launch truck simulator.yaml` - run simulation (gazebo + part of pipeline), default port 11345.

### Gazebo 11
We use [Gazebo 11](https://classic.gazebosim.org) as physics engine (namely ode).
The simulator is integrated at the container (server side).
But you have to install gazebo on your client machine by yourself for rendering purpouses (follow [guide](https://classic.gazebosim.org/tutorials?tut=install_from_source)).

```
# default port is 11345, on client
export GAZEBO_MODEL_PATH=/path/to/repo/truck/packages/truc_gazebo/models
gzclient --verbose
```

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

## Gamepad layout
- (❌) Cross button — switch to OFF mode (stop immediately, disable motor)
- (⭕) Circle button — switch to REMOTE mode (motion controlled via gamepad)
- (🔺) Triangle button — switch to AUTO mode (motion controlled programmatically)
- Left stick, horizontal axis — controls steering (left-right)
- Right stick, vertical axis — controls velocity (forward/backward)

## Foxglove
TODO(@dasimagin)
