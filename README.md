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

Now you can start a new container and attach it. 

```
# dev/amd64
docker-compose up -d truck-base-amd64

# jetson
docker-compose up -d truck-base-jetson

# attach to running container
# use ctrl+p+q to detach
docker attach truck-base

# or just run another shell
# exit doesn't stop the container
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

### Local container development

We probably shouldn't use `ansible` here... Todo: rewrite to something simpler, like `make`.


Look inside `local-playbook.yaml` for more info on commands

#### Build packages
```bash
ansible-playbook local-playbook.yaml --tags build
```

#### Start nodes
```bash
ansible-playbook local-playbook.yaml --tags start
```

After starting nodes logs are written into `log` directory.

#### Stop nodes
```bash
ansible-playbook local-playbook.yaml --tags stop
```

Stopping active nodes requires pid files from `log` directory.


## Foxglove visualization

1. Download and install Foxglove Studio app from https://foxglove.dev/
2. Start your packages and rosbridge_server (for example, with `ansible` as listed above)
3. In Foxglove Studio open new connection to Rosbridge with address and port of your container.
4. Add some panels, e.g. 3D panel, Raw Messages panel.
