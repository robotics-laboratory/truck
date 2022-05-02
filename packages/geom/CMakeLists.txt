cmake_minimum_required(VERSION 3.5)
project(geom)

find_package(ament_cmake REQUIRED)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

file(GLOB SRCS src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SRCS})

target_include_directories(geom
  PUBLIC
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

ament_package()