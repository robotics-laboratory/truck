cmake_minimum_required(VERSION 3.5)
project(perf)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(truck_interfaces REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_executable(perf_stat_node src/node.cpp src/stat.cpp src/main.cpp)

ament_target_dependencies(perf_stat_node rclcpp truck_interfaces)

target_include_directories(perf_stat_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(
    TARGETS perf_stat_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
