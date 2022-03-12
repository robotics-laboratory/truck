cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_node)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pure_pursuit_msgs REQUIRED)
find_package(planning_interfaces REQUIRED)

file(GLOB SRC
  src/*.cpp
)

add_executable(node
   ${SRC}
)

ament_target_dependencies(node rclcpp pure_pursuit_msgs planning_interfaces)

install(TARGETS
  node
  DESTINATION lib/${PROJECT_NAME}
)

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
