cmake_minimum_required(VERSION 3.5)
project(visualization)

find_package(ament_cmake REQUIRED)
find_package(model REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(truck_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_executable(visualization_node src/main.cpp src/visualization_node.cpp)

ament_target_dependencies(visualization_node rclcpp model nav_msgs
    truck_interfaces visualization_msgs)

install(
    TARGETS visualization_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
