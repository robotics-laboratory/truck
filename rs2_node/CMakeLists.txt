cmake_minimum_required(VERSION 3.5)
project(rs2_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(rs2_node main.cpp)
ament_target_dependencies(rs2_node rclcpp sensor_msgs OpenCV cv_bridge realsense2)
target_link_libraries(rs2_node realsense2)

install(TARGETS
  rs2_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
