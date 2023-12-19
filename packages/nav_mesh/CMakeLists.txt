cmake_minimum_required(VERSION 3.8)
project(nav_mesh)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(map REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

add_library(
    ${PROJECT_NAME} SHARED
    src/nav_mesh.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}
    common
    geom
    map
    OpenCV
    CGAL
    Boost
)

target_include_directories(
    ${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
    common
    geom
    map
    OpenCV
    CGAL
    Boost
)

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
