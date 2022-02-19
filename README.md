# Truck



## Getting started
### Docker
We use docker for two purposes:
- Unified isolated environment for easy development and tests
- Dependency and code delivery system

All prebuild base images are stored at our registry ```cr.yandex/crp8hpfj5tuhlaodm4dl```. At this moment only ubuntu 18.04 is supported (images has some differences):
- **truck-base-jetson** (jetson runtime)
- **truck-base-amd64** (dev env, without cuda)

#### Pull and Run
For jetson check that nvidia runtime is enbled by default in ```/etc/docker/daemon.json```.

```
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
```

Now you can start new container and attach. 

```
# dev/amd64
docker-compose up -d truck-base-amd64

# jetson
docker-compose up -d truck-base-jetson

# attach to running container
# use ctrl+p+q to detach
docker attach truck-base

# run another shell
# you can safely exit it
# and it won't stop the container
docker exec -it truck-base bash

# stop container
docker stop truck-base
```

#### Build and Push
If you need to build container, follow this steps. Up version in ```docker-compose.yaml``` if orresponding dockerfile is changed.

```
# dev/amd64
docker-compose build truck-base-amd64
docker-compose push truck-base-amd64

# jetson
docker-compose build truck-base-jetson
docker-compose push truck-base-jetson
```

#### Start simulation

```
# Build and source the setup files

# Start the simulation with ros2 launch robot rviz.launch.py
