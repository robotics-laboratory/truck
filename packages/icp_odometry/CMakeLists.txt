cmake_minimum_required(VERSION 3.8)
project(icp_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost COMPONENTS thread filesystem system program_options date_time chrono timer serialization REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(truck_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

add_executable(icp_odometry_node
  src/main.cpp
  src/icp_odometry_node.cpp
  src/conversion.cpp
)

ament_target_dependencies(icp_odometry_node common geom libpointmatcher rclcpp sensor_msgs Boost truck_interfaces)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(
    TARGETS icp_odometry_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch config
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
