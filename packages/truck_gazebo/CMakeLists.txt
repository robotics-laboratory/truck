cmake_minimum_required(VERSION 3.8)
project(truck_gazebo)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

install(
    DIRECTORY worlds launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
