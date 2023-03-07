cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(geom REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(model REQUIRED)
find_package(rclcpp REQUIRED)
find_package(truck_interfaces REQUIRED)

add_executable(
    pure_pursuit_node
    src/controller.cpp
    src/main.cpp
    src/util.cpp
)

ament_target_dependencies(
    pure_pursuit_node
    geom
    geometry_msgs
    model
    nav_msgs
    rclcpp
    truck_interfaces
    visualization_msgs
)

install(
    TARGETS pure_pursuit_node
    DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(
    pure_pursuit_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
