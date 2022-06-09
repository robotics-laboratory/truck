cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(truck_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(model REQUIRED)
find_package(tf2 REQUIRED)
find_package(geom REQUIRED)

set(COMMON_SRC src/controller.cpp src/speed_planner.cpp src/curvature_planner.cpp)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  ${COMMON_SRC}
)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp truck_interfaces model geom visualization_msgs nav_msgs geometry_msgs)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

add_library(${PROJECT_NAME}_simulator
  src/simulator.cpp
  ${COMMON_SRC}
)

set_target_properties(${PROJECT_NAME}_simulator PROPERTIES PUBLIC_HEADER include/pure_pursuit/simulator.hpp)

target_include_directories(${PROJECT_NAME}_simulator PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

set(SIMULATOR_DEPENDENCIES rclcpp tf2 truck_interfaces visualization_msgs nav_msgs geometry_msgs model geom)

ament_target_dependencies(${PROJECT_NAME}_simulator ${SIMULATOR_DEPENDENCIES})
ament_export_dependencies(${SIMULATOR_DEPENDENCIES})

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}_simulator")
ament_export_libraries(${PROJECT_NAME}_simulator)

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME}_simulator)

install(
  TARGETS ${PROJECT_NAME}_simulator EXPORT ${PROJECT_NAME}_simulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}_simulator
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Add a gtest named ${PROJECT_NAME}_my_test
  add_subdirectory("test")
endif()

ament_package()
