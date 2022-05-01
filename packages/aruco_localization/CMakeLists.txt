cmake_minimum_required(VERSION 3.8)
project(aruco_localization)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(aruco_localization src/aruco_localization.cpp src/math_helpers.cpp src/coordinator.cpp)
target_include_directories(aruco_localization PRIVATE include/aruco_localization)
ament_target_dependencies(aruco_localization rclcpp nav_msgs sensor_msgs OpenCV cv_bridge visualization_msgs tf2_msgs)

install(TARGETS
    aruco_localization
    DESTINATION lib/${PROJECT_NAME})

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # uncomment the line when a copyright and license is not present in all source files
#  #set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # uncomment the line when this package is not in a git repo
#  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
