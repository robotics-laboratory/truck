cmake_minimum_required(VERSION 3.8)
project(planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geom REQUIRED)
find_package(model REQUIRED)
find_package(collision REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)

add_executable(${PROJECT_NAME}_node src/main.cpp src/planner_node.cpp
                                    src/search.cpp)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  geom
  model
  collision
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  Boost)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

target_include_directories(
  ${PROJECT_NAME}_node
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
