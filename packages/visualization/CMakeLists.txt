cmake_minimum_required(VERSION 3.5)
project(visualization)

find_package(ament_cmake REQUIRED)
find_package(model REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(map REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(
    visualization_node
    src/color.cpp
    src/main.cpp
    src/visualization_node.cpp)

ament_target_dependencies(
    visualization_node
    rclcpp model
    nav_msgs
    truck_msgs
    map
    visualization_msgs
    tf2_geometry_msgs)

install(
    TARGETS visualization_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
