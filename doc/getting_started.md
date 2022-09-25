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
docker-compose up -d truck-amd64

# jetson
docker-compose up -d truck-jetson

# just run another shell (preferred)
# exit doesn't stop the container
docker exec -it truck bash

# attach to running container
# use ctrl+p+q to detach
docker attach truck

# stop container
docker stop truck
```
### Build and push
If you need to build container, follow this steps. Up version in ```docker-compose.yaml``` if orresponding dockerfile is changed.

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
To build all packages

```bash
colcon build --merge-install
```

To build you some single package

```bash
colcon build --merge-install --packages-up-to package_name
```

Current installation dir is ```/truck/packages/install```. To run setup do

```bash
source install/setup.bash
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


