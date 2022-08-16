cmake_minimum_required(VERSION 3.5)
project(control_proxy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(model REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(truck_interfaces REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_executable(control_proxy src/main.cpp src/control_proxy.cpp)

ament_target_dependencies(control_proxy model rclcpp sensor_msgs truck_interfaces)

install(
    TARGETS control_proxy
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
