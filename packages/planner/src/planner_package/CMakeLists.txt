cmake_minimum_required(VERSION 3.8)
project(planner_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(planner_interfaces REQUIRED)

add_executable(oc_grid_executable src/occupancy_grid.cpp src/global_variables.cpp)
add_executable(dist_trans_executable src/distanse_transform.cpp src/global_variables.cpp)
add_executable(graph_executable src/graph_visualization.cpp src/global_variables.cpp src/a_star.cpp)

ament_target_dependencies(oc_grid_executable rclcpp OpenCV nav_msgs)
ament_target_dependencies(dist_trans_executable rclcpp OpenCV nav_msgs visualization_msgs std_msgs)
ament_target_dependencies(graph_executable rclcpp sensor_msgs PCL nav_msgs planner_interfaces)

install(TARGETS oc_grid_executable DESTINATION lib/${PROJECT_NAME})
install(TARGETS dist_trans_executable DESTINATION lib/${PROJECT_NAME})
install(TARGETS graph_executable DESTINATION lib/${PROJECT_NAME})

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