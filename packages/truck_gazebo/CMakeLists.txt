cmake_minimum_required(VERSION 3.8)
project(truck_gazebo)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(example_position examples/example_position.cpp)
ament_target_dependencies(example_position
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(example_velocity examples/example_velocity.cpp)
ament_target_dependencies(example_velocity
  rclcpp
  rclcpp_action
  control_msgs
)

install(
    DIRECTORY worlds launch
    DESTINATION share/${PROJECT_NAME})

install(TARGETS example_position example_velocity
    DESTINATION lib/${PROJECT_NAME}
)  

ament_package()
