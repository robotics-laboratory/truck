cmake_minimum_required(VERSION 3.8)
project(waypoint_follower)

find_package(ament_cmake REQUIRED)
find_package(collision REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(model REQUIRED)
find_package(motion REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(speed REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(truck_msgs REQUIRED)

add_executable(
  ${PROJECT_NAME}_node src/waypoint_follower.cpp src/main.cpp
                       src/waypoint_follower_node.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  collision
  common
  geom
  geometry_msgs
  nav_msgs
  model
  rclcpp
  speed
  truck_msgs
  tf2
  tf2_ros
  std_srvs
  visualization_msgs
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

target_include_directories(
  ${PROJECT_NAME}_node
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
