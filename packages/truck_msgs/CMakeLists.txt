cmake_minimum_required(VERSION 3.8)
project(truck_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/Control.msg
  msg/ControlMode.msg
  msg/CpuStat.msg
  msg/HardwareStatus.msg
  msg/HardwareTelemetry.msg
  msg/IcpOdometryStat.msg
  msg/MemStat.msg
  msg/MetaData.msg
  msg/NavigationMesh.msg
  msg/NavigationRoute.msg
  msg/PerfStat.msg
  msg/Point.msg
  msg/PurePursuitStatus.msg
  msg/SimulationState.msg
  msg/Trajectory.msg
  msg/TrajectoryState.msg
  msg/Waypoints.msg
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  nav_msgs
  visualization_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
