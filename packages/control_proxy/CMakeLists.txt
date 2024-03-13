cmake_minimum_required(VERSION 3.5)
project(control_proxy)

find_package(ament_cmake REQUIRED)
find_package(common)
find_package(model REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(${PROJECT_NAME}_node src/main.cpp src/control_proxy_node.cpp)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  common
  model
  rclcpp
  sensor_msgs
  std_srvs
  truck_msgs
  yaml_cpp_vendor
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
