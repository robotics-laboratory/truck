cmake_minimum_required(VERSION 3.5)
project(truck_gazebo_plugins)

find_package(ament_cmake REQUIRED)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

file(GLOB SRCS src/*.cpp)

add_library(truck_gazebo_plugins SHARED ${SRCS})
ament_target_dependencies(truck_gazebo_plugins gazebo_dev gazebo_ros nav_msgs rclcpp)

target_include_directories(truck_gazebo_plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_export_targets(truck_gazebo_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(truck_gazebo_plugins gazebo_dev gazebo_ros nav_msgs rclcpp)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS truck_gazebo_plugins
  EXPORT truck_gazebo_plugins
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_file(CLANG_FORMAT .clang-format PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../..)
  ament_clang_format(CONFIG_FILE ${CLANG_FORMAT})
endif()

ament_package()
