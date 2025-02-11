# 2D LiDAR Map Builder

This package allows to build 3D LiDAR map of environment by providing bag file of `.mcap` format with recorded odometry and LiDAR topics.

In the recorded bag robot should move calmly and visit all parts of environment from different angles, it's important to make several loop-closures while riding.

## How to use

Execute next commands inside `truck/packages/lidar_map` directory.

Run executable `lidar_map_executable` with `--help` command to explore available options and arguments:
```console
ros2 run lidar_map lidar_map_executable --help
```

Parameters:

1. `--mcap-input`: path to .mcap file with a ride (required)

2. `--mcap-output`: path to NON-EXISTING folder for saving map (required)

3. `--mcap-log`: path to NON-EXISTING folder for saving map logs (optional)

4. `--json-log`: path to json file for saving map logs (optional)

Run the executable `lidar_map_executable` with the assigned `--mcap-input` and `--mcap-output` arguments:
```console
ros2 run lidar_map lidar_map_executable
  --mcap-input data/rides/ride_atrium_XX_YY_ZZ.mcap
  --mcap-output data/clouds/cloud_atrium_XX_YY_ZZ
```


### Logging Options

You can now specify logging options separately:

1. `.mcap` file: contains poses and clouds for each iteration of pose graph

2. `.json` file: contains poses with ICP and odom edges on each iteration of the graph

```console
ros2 run lidar_map lidar_map_executable
  --mcap-input data/rides/ride_atrium_XX_YY_ZZ.mcap
  --mcap-output data/clouds/cloud_atrium_XX_YY_ZZ
  --mcap-log data/logs/mcap/cloud_atrium_XX_YY_ZZ_log
  --json-log data/logs/json/cloud_atrium_XX_YY_ZZ_log.json
```
### Debug

To visualize the normals, add this to main.cpp.

```c++
//Calculate attributes for the reference cloud using i and next cloud
auto cloud_with_attributes = builder.calculateAttributesForReferenceCloud(clouds[i], clouds[i + 1]);
mcap_writer->writeCloudWithAttributes(
        "data/logs/icp_logs", // path where the mcap file will be saved
        cloud_with_attributes,
        70, // percentage of normals to be visualized
        true, // true to write outliers
        true, // true to write normals
        "world"); // frame name
```
