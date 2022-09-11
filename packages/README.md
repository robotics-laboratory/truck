# ROS2 Packages

- [control_proxy](control_proxy/README.md)
- [model](model/README.md)
- [truck_gazebo](truck_gazebo/README.md)
- [truck_gazebo_plugins](truck_gazebo_plugins/README.md)
- [truck](truck/README.md)

## Build

To build all packages

```bash
colcon build --merge-install
```

To build you some single package

```bash
colcon build --merge-install --packages-up-to package_name
```

## Install
Current installation dir is ```/truck/packages/install```. To run setup do

```bash
. install/setup.bash
```
