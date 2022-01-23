# Truck

## Getting started
### Docker
We use docker for two purposes:
- Unified isolated environment for easy development and tests
- Dependency and code delivery system

All prebuild base images are stored at our registry. At this moment only ubuntu 18.04 is supported (images has some differences):
- **truck-base-jetson** (jetson runtime)
- **truck-base-amd64** (dev env, without cuda)

You may add our registry name to variable.

```REGESTRY="cr.yandex/crp8hpfj5tuhlaodm4dl"```

#### Pull and Run

```
# For dev/amd64

docker run -it --rm -d \
    --port 9090:9090 \
    --name truck \
    $REGESTRY/truck-base:amd64-latest \
    /bin/bash

# For jetson

docker run -it --rm -d \
    --privelged \
    --runtime nvidia \
    --port 9090:9090 \
    --volumes /dev:/dev \
    --name truck \
    $REGESTRY/truck-base:jetson-latest \
    /bin/bash

# Attach

docker attach truck
```

#### Build and Push
```
VERSION="0.1.0"

docker build \
    -f docker/truck-base-amd64.dockerfile \
    -t $REGESTRY/truck-base:amd64-$VERSION \
    .

docker push cr.yandex/crp8hpfj5tuhlaodm4dl/truck-base:amd64-$VERSION

docker tag \
    $REGISTRY/truck-base:amd64-$VERSION \
    $REGISRTY/truck-base:amd64-latest

docker push $RGESTRY/truck-base:amd64-latest
```

