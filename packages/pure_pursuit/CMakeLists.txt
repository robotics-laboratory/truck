cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(geom REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(model REQUIRED)
find_package(motion REQUIRED)
find_package(rclcpp REQUIRED)
find_package(truck_msgs REQUIRED)

add_executable(
    ${PROJECT_NAME}_node
    src/pure_pursuit.cpp
    src/pure_pursuit_node.cpp
    src/main.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    geom
    geometry_msgs
    model
    motion
    nav_msgs
    rclcpp
    truck_msgs
    visualization_msgs
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
