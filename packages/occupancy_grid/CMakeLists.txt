cmake_minimum_required(VERSION 3.8)
project(occupancy_grid)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

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

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

add_executable(occupancy_grid_node
  src/main.cpp
  src/occupancy_grid_node.cpp
)

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
  tf2_geometry_msgs)

target_include_directories(${PROJECT_NAME}_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(
    TARGETS occupancy_grid_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
