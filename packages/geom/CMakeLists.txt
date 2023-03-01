cmake_minimum_required(VERSION 3.5)
project(geom)

find_package(ament_cmake REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(Boost REQUIRED)
find_package(common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/angle.cpp
    src/arc.cpp
    src/bezue.cpp
    src/circle.cpp
    src/distance.cpp
    src/line.cpp
    src/motion.cpp
    src/segment.cpp
    src/transform.cpp
    src/vector.cpp
)

ament_target_dependencies(geom common nav_msgs geometry_msgs)

target_include_directories(geom PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_export_targets(geom HAS_LIBRARY_TARGET)
ament_export_dependencies(common nav_msgs geometry_msgs)

install(
    TARGETS geom
    EXPORT geom
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory("test")
endif()

ament_package()