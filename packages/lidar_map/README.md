# 2D LiDAR Map Builder

This package allows to build 2D LiDAR map of environment by providing bag file of `.mcap` format with recorded odometry and LiDAR topics.

In the recorded bag robot should move calmly and visit all parts of environment from different angles, it's important to make several loop-closures while riding.

## How to use

Execute next commands inside `truck/packages/lidar_map` directory.

Parameters:

1. `--mcap-input`, `mi`: Path to the .mcap file with ride data (required).

2. `--mcap-output`, `mo`: Path to the folder where the .mcap file with the 2D LiDAR map will be stored (folder should not exist, required).

3. `--mcap-log`, `ml`: Path to the folder for .mcap logs (optional).

4. `--json-log`, `jl`: Path to the .json log file (optional).

Run executable `lidar_map_executable` with `--help` command to explore available options and arguments:
```console
ros2 run lidar_map lidar_map_executable --help
```

Run the executable `lidar_map_executable` with the assigned `--mcap-input` and `--mcap-output` arguments:
```console
ros2 run lidar_map lidar_map_executable 
  --mcap-input data/rides/ride_real_office.mcap 
  --mcap-output data/clouds/cloud_real_office
```


### Logging Options

You can now specify logging options separately:

1. .mcap file: Contains poses and clouds for each iteration of the pose graph, along with the final map.

2. .json file: Contains poses with edges and graphs for ICP edges for each iteration of the pose graph.

```console
ros2 run lidar_map lidar_map_executable 
  --mcap-input data/rides/ride_sim_map_7.mcap 
  --mcap-output data/clouds/sim_map_7 
  --mcap-log data/logs/mcap_logs 
  --json-log data/logs/json_log.json
```
