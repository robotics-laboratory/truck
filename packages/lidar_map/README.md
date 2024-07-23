# 2D Point Cloud Map Builder

This package allows to build 2D point cloud maps of environment by providing bag file with recorded odometry and LiDAR data.

## Overview

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

## Instruction

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
