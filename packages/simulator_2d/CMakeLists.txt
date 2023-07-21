cmake_minimum_required(VERSION 3.5)
project(simulator_2d)

find_package(ament_cmake REQUIRED)
find_package(model REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(model REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(geom REQUIRED)
find_package(control_proxy REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization REQUIRED)
find_package(visualization_msgs REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_executable(
    ${PROJECT_NAME}_node
    src/main.cpp
    src/simulator_node.cpp
    src/simulator_engine.cpp)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    rclcpp
    model
    nav_msgs
    truck_msgs
    geom
    control_proxy
    tf2_geometry_msgs
    tf2_msgs)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

target_include_directories(
    ${PROJECT_NAME}_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_package()
