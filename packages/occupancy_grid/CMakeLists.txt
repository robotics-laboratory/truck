cmake_minimum_required(VERSION 3.8)
project(occupancy_grid)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geom REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(occupancy_grid_node src/main.cpp src/occupancy_grid_node.cpp)

ament_target_dependencies(
  occupancy_grid_node
  common
  cv_bridge
  geom
  image_geometry
  image_transport
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
)

target_include_directories(
  ${PROJECT_NAME}_node
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

install(TARGETS occupancy_grid_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
