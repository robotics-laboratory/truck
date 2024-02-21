cmake_minimum_required(VERSION 3.8)
project(routing)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(map REQUIRED)
find_package(navigation REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(
    ${PROJECT_NAME}_node
    src/main.cpp
    src/routing_node.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    rclcpp
    map
    navigation
    truck_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    Boost
    Eigen3
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(
    ${PROJECT_NAME}_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
endif()

ament_package()
