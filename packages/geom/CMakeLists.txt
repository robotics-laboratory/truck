cmake_minimum_required(VERSION 3.5)
project(geom)

find_package(ament_cmake REQUIRED)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(geom
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(geom)

# Export modern CMake targets
ament_export_targets(geom)

install(
  TARGETS geom
  EXPORT geom
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Add a gtest named ${PROJECT_NAME}_my_test
  add_subdirectory("test")
endif()

ament_package()