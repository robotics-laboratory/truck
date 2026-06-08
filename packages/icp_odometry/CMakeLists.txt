cmake_minimum_required(VERSION 3.8)
project(icp_odometry)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(truck_msgs REQUIRED)

find_package(libpointmatcher REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(icp_odometry_node
  src/main.cpp
  src/icp_odometry_node.cpp
  src/conversion.cpp
)

target_include_directories(icp_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(icp_odometry_node
  common
  geom
  sensor_msgs
  std_msgs
  nav_msgs
  rclcpp
  truck_msgs
)

target_link_libraries(icp_odometry_node
  ${libpointmatcher_LIBRARIES}
  yaml-cpp
)

install(TARGETS
  icp_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
