cmake_minimum_required(VERSION 3.8)
project(istk_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(rear_sector_filter_node src/rear_sector_filter_node.cpp)
ament_target_dependencies(
  rear_sector_filter_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(rear_sector_filter_node yaml-cpp)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/update_laser_transform.py
  scripts/dynamic_tf_s2.py
  scripts/laser_transform_control.py
  scripts/rear_sector_filter.py
  scripts/hwnode.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS rear_sector_filter_node DESTINATION lib/${PROJECT_NAME})

ament_package()
