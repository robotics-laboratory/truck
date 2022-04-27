cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_node)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pure_pursuit_msgs REQUIRED)
find_package(planning_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

file(GLOB SRC
  src/*.cpp
)

add_executable(${PROJECT_NAME}
   ${SRC}
)

ament_target_dependencies(${PROJECT_NAME} rclcpp pure_pursuit_msgs planning_interfaces visualization_msgs nav_msgs geometry_msgs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
