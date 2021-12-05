# Truck

# Getting started

## Jetson 
Jetson Xavier NX is our brain. Follow this [instruction](https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit) to set up board (also avaible [version](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#intro) for Jetson Nano).

## Docker
All development is done inside the docker container. First of all you need build it localy.
```
# for dev machine
docker build -t truck-image .

# for jetson
docker build -t truck-image --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.5.0 .
```

Than you may run container, using following command.

```
docker run -it --rm truck-image /bin/bash
```
