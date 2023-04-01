cmake_minimum_required(VERSION 3.8)
project(truck_interfaces)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  msg/Control.msg
  msg/ControlMode.msg
  msg/CpuStat.msg
  msg/HardwareStatus.msg
  msg/HardwareTelemetry.msg
  msg/MemStat.msg
  msg/MetaData.msg
  msg/Path.msg
  msg/PerfStat.msg
  msg/Point.msg
  msg/RandomSeed.msg
  msg/Scene.msg
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
