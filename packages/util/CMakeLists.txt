cmake_minimum_required(VERSION 3.8)
project(util)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(util INTERFACE)

target_include_directories(util INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/util>"
)

install(
  TARGETS util EXPORT util
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

target_link_libraries(util INTERFACE yaml-cpp)

ament_export_dependencies(yaml-cpp)

# Export old-style CMake variables
ament_export_include_directories("include/util")

# Export modern CMake targets
ament_export_targets(util)

install(
  DIRECTORY include/
  DESTINATION include/util
)

ament_package()
