# ROS2 Packages
Home of all our packages!

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
