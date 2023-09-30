cmake_minimum_required(VERSION 3.8)
project(tree_planner)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(collision REQUIRED)
find_package(fastgrid REQUIRED)
find_package(geom REQUIRED)
find_package(model REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED)

add_executable(
    ${PROJECT_NAME}_node
    src/tree_planner.cpp
    src/tree_planner_node.cpp
    src/main.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    rclcpp
    collision
    fastgrid
    geom
    model
    tf2
    tf2_ros
    visualization_msgs
    Boost
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

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
endif()

ament_package()