cmake_minimum_required(VERSION 3.8)
project(simulator_2d)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(
    ${PROJECT_NAME}_node
    src/main.cpp
    src/demo.cpp)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    rclcpp
    truck_msgs
    visualization_msgs)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME})

target_include_directories(
    ${PROJECT_NAME}_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_package()
