cmake_minimum_required(VERSION 3.5)
project(common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_library(${PROJECT_NAME} SHARED src/empty.cpp)
ament_target_dependencies(common rclcpp)

target_include_directories(common PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_export_targets(commonTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(
  TARGETS common
  EXPORT commonTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/common
)

ament_package()
