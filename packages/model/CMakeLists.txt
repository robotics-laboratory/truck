cmake_minimum_required(VERSION 3.8)
project(model)

find_package(ament_cmake REQUIRED)
find_package(util REQUIRED)

add_library(model INTERFACE)

target_include_directories(model INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/model>"
)

install(
  TARGETS model EXPORT model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_target_dependencies(model INTERFACE util)

ament_export_dependencies(util)

# Export old-style CMake variables
ament_export_include_directories("include/model")

# Export modern CMake targets
ament_export_targets(model)

install(
  DIRECTORY include/
  DESTINATION include/model
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
