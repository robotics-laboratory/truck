cmake_minimum_required(VERSION 3.5)
project(control_proxy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(model REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_executable(control_proxy_node src/main.cpp src/control_proxy_node.cpp)

ament_target_dependencies(control_proxy_node model rclcpp sensor_msgs truck_msgs yaml_cpp_vendor)

install(
    TARGETS control_proxy_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

ament_package()
