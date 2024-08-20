# 2D LiDAR Map Builder

This package allows to build 2D LiDAR map of environment by providing bag file of `.mcap` format with recorded odometry and LiDAR topics.

In the recorded bag robot should move calmly and visit all parts of environment from different angles, it's important to make several loop-closures while riding.

## How to use

Execute next commands inside `truck/packages/lidar_map` directory.

Run executable `lidar_map_executable` with `--help` command to explore available options and arguments:
```console
ros2 run lidar_map lidar_map_executable --help
```

Run executable `lidar_map_executable` with assigned `--input` and `--output` arguments:
```console
ros2 run lidar_map lidar_map_executable \
  --input data/rides/ride_real_office.mcap \
  --output data/clouds/cloud_real_office
```

If you building 2D LiDAR map based on simulated data, then you can measure its quality by comparing it with corresponding ground truth vector map. Assign `--test` argument with the name of one of the existing vector maps located in the `packages/map/data` folder:
```console
ros2 run lidar_map lidar_map_executable \
  --input data/rides/ride_sim_map_7.mcap \
  --output data/clouds/sim_map_7 \
  --test map_7.geojson
```
