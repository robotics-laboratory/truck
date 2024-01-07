# Truck

**Truck** is an open source project dedicated to the construction and development of the 2.5D indoor autonomous vehicle based on the [ackermann steering model](https://github.com/robotics-laboratory/truck/blob/master/doc/ackermann_vehicle.md). It's maintained by the Robotics Group (Faculty of Computer Science, HSE University) and enthusiasts. You may follow our progress on [YouTube](https://www.youtube.com/watch?v=hF6cDalz8-I&list=PLR1nN_AQOO9zHpkW-phZnqVywjUCj7zHZ).

It is intended to be an educational reasearch project, so, among our goals is to maintain a rich set of study materials and detailed [documentation](https://github.com/robotics-laboratory/truck/blob/master/doc/README.md).

## Get Started. How to launch container

### UNIX-like (Linux/MacOS)
```
docker compose up -d
```

### Windows
```
cmd /C 'set "USER=%USERNAME%" && docker compose up -d'
```

\
Then you can attach to the container, its name will look like truck-&lt;your username&gt;
```
docker exec -it <container name> bash
```
