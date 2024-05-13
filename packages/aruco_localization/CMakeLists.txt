cmake_minimum_required(VERSION 3.8)
project(aruco_localization)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(
  aruco_localization
  src/aruco_localization.cpp
  src/math_helpers.cpp
  src/camera_tracker.cpp
  src/transform.cpp
  src/visualization_helpers.cpp
  src/tf_graph.cpp
  src/graph_algorithms.cpp)

target_include_directories(aruco_localization
                           PRIVATE include/aruco_localization)
ament_target_dependencies(
  ${PROJECT_NAME}
  cv_bridge
  nav_msgs
  OpenCV
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  visualization_msgs)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
