# Docker
We use docker for two purposes:
- Unified isolated environment for easy development and tests
- Dependency and code delivery system

## Getting started
At this moment only ubuntu 18.04 containers supported (jetson has specific env, so images has differences):
- **truck-base:jetson-latest** (jetson runtime with cuda)
- **truck-base:amd64-latest** (dev env without cuda)

All prebuild base images are stored at registry ```cr.yandex/crp8hpfj5tuhlaodm4dl```.
So, you can easyly pull it.

```
docker pull cr.yandex/crp8hpfj5tuhlaodm4dl/truck-base:dev-latest
```

Example of test run (nvidia runtime is enabled by default on jetson).
```
docker run --it --rm cr.yandex/crp8hpfj5tuhlaodm4dl/truck-base:dev-latest /bin/bash
```

## Build and pull
Example of build for dev image.
```
docker build --pull -f docker/Dockerfile.amd64 . \
    -t cr.yandex/crp8hpfj5tuhlaodm4dl/truck-base:amd64-$(date '+%y%m%d-%H%M')
```
