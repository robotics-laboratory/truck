cmake_minimum_required(VERSION 3.8)
project(truck_foxglove)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(odom_translator src/main.cpp src/odom_translator.cpp)
ament_target_dependencies(odom_translator tf2 tf2_geometry_msgs rclcpp rmw std_msgs nav_msgs visualization_msgs)

target_include_directories(odom_translator PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(TARGETS odom_translator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
