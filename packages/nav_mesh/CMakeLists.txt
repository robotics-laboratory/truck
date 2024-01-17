cmake_minimum_required(VERSION 3.8)
project(nav_mesh)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(map REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

add_library(
    ${PROJECT_NAME}_builder SHARED
    src/nav_mesh_builder.cpp
)

add_library(
    ${PROJECT_NAME}_viewer SHARED
    src/nav_mesh_viewer.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_builder
    common
    geom
    map
    CGAL
    Boost
)

ament_target_dependencies(
    ${PROJECT_NAME}_viewer
    common
    geom
    OpenCV
)

target_include_directories(
    ${PROJECT_NAME}_builder PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_include_directories(
    ${PROJECT_NAME}_viewer PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)

ament_export_dependencies(
    common
    geom
    map
    CGAL
    Boost
    OpenCV
)

install(
    TARGETS ${PROJECT_NAME}_builder ${PROJECT_NAME}_viewer
    EXPORT ${PROJECT_NAME}_targets
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
