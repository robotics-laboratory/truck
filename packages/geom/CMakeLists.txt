cmake_minimum_required(VERSION 3.5)
project(geom)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

add_library(
    ${PROJECT_NAME} SHARED
    src/angle.cpp
    src/angle_vector.cpp
    src/arc.cpp
    src/bezier.cpp
    src/circle.cpp
    src/complex_polygon.cpp
    src/distance.cpp
    src/line.cpp
    src/msg.cpp
    src/polygon.cpp
    src/polyline.cpp
    src/pose.cpp
    src/ray.cpp
    src/segment.cpp
    src/transform.cpp
    src/vector.cpp
)

ament_target_dependencies(${PROJECT_NAME} common geometry_msgs nav_msgs tf2 CGAL Boost)

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(common geometry_msgs nav_msgs tf2 CGAL Boost)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")

    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory("test")
endif()

ament_package()
