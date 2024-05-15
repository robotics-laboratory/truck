# Foxglove

## Getting started
We use [foxglove](https://foxglove.dev) as visualization tool for our project. Download [Foxglove studio](https://foxglove.dev/download) to your machine or jetson. And look at official [docs](https://foxglove.dev/docs/studio).

At current moment we use [rosbridge](http://wiki.ros.org/rosbridge_suite) to communicate with ROS2, so use this connection method at live robot.
- For remote access use ```ws://jetson.local:9090``` (recomended to connect jetson as wifi hotspot).
- For local run just use address ```ws://localhost:9090```.

## Layouts
We have several launch configs in package 'truck'. Some layout is prepared for each of them. Freely use it, exporting from repo to foxglove studio.
