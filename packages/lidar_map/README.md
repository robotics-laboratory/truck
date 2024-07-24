# 2D Lidar Map Builder

This package allows to build 2D LiDAR map of environment by providing bag file of `.mcap` format with recorded odometry and LiDAR topics.

In the recorded bag robot should move calmly and visit all parts of environment from different angles, it's important to make several loop-closures while riding.

## Overview

Hierarchy of the `lidar_map` package:
```
└── lidar_map
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

All bags are stored in `data` folder. Bags in `data/rides` folder are so called ride bags which contain recorded odometry and LiDAR topics. Bags in `data/clouds` folder are bags which contain built 2D LiDAR maps based on corresponding ride bags.

For example, by taking ride bag `data/rides/ride_real_office.mcap` we created 2D LiDAR map which will be stored in folder `data/clouds/cloud_real_office`.

## How to use

Build `lidar_map` package:
```console
root@fca8223dfa98:/truck# make build packages="lidar_map"
```

Source installed `lidar_map` package to make it available for ROS:
```console
root@fca8223dfa98:/truck# source install/setup.sh
```

Than go to the `truck/packages/lidar_map` directory for convinience.

Now you can run executable `lidar_map_executable`. At first run `--help` command to explore available options and arguments:
```console
root@fca8223dfa98:/truck/packages/lidar_map# ros2 run lidar_map lidar_map_executable --help
```

Run executable `lidar_map_executable` with assigned options and arguments:
```console
root@fca8223dfa98:/truck/packages/lidar_map# ros2 run lidar_map lidar_map_executable
  --input data/rides/ride_real_office.mcap
  --output data/clouds/cloud_real_office
```

If you building 2D LiDAR map based on simulated data, then you can measure its quality by comparing it with corresponding ground truth vector map. You can do this with `--test` option:
```console
root@fca8223dfa98:/truck/packages/lidar_map# ros2 run lidar_map lidar_map_executable
  --input data/rides/ride_sim_map_7.mcap
  --output data/clouds/sim_map_7
  --test
```