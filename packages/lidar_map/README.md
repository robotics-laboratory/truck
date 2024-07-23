# 2D Point Cloud Map Builder Library

This package allows to build 2D point cloud map of environment by providing bag file of `.mcap` format with recorded odometry and LiDAR topics.

Requirements for bag file
- ride should be calm and long enough
- robot should visit all parts of environment from different angles
- have several loop-closures

## Overview

Hierarchy of the `lidar_map` library:
```
└── lidar_map
    ├── cmake_modules
    └── data
        └── clouds
            └── cloud_real_office
                ├── cloud_real_office_0.mcap
                └── metadata.yaml
        └── rides
            ├── ride_real_office.mcap
            └── ride_sim_map_7.mcap
    ├── include
    ├── src
    ├── CMakeLists.txt
    └── package.xml
```

All bags stored in `data` folder. Bags in `data/rides` folder are so called "ride bags" with recorded odometry and LiDAR topics. Bags in `data/clouds` folder are bags with built 2D point cloud maps based on corresponding ride bag.

## Build and run

Build `lidar_map` package:
```console
root@fca8223dfa98:/truck# make build packages="lidar_map"
```

Source installed `lidar_map` package to make it available for ROS:
```console
root@fca8223dfa98:/truck# source install/setup.sh
```

Now you can run executable `lidar_map_executable` with `--help` command to explore available options and arguments:
```console
root@fca8223dfa98:/truck# ros2 run lidar_map lidar_map_executable --help
```

Go to the `truck/packages/lidar_map` directory and run executable `lidar_map_executable` with assigned options and arguments:
```console
root@fca8223dfa98:/truck/packages/lidar_map# ros2 run lidar_map lidar_map_executable
  --input data/rides/ride_real_office.mcap
  --output data/clouds/cloud_real_office
```

If you have building lidar map based on simulation data, then you can compare it with corresponding ground truth vector map by `--test` option
```console
root@fca8223dfa98:/truck/packages/lidar_map# ros2 run lidar_map lidar_map_executable
  --input data/rides/ride_sim_map_7.mcap
  --output data/clouds/sim_map_7
  --test
```
