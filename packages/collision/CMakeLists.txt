cmake_minimum_required(VERSION 3.8)
project(collision)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(fastgrid REQUIRED)
find_package(geom REQUIRED)
find_package(map REQUIRED)
find_package(model REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_library(
    ${PROJECT_NAME} SHARED
    src/collision_checker.cpp
    src/map.cpp
)

ament_target_dependencies(
    collision
    common
    fastgrid
    geom
    map
    model
    nav_msgs
    OpenCV
)

target_include_directories(
    collision PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_export_targets(collision HAS_LIBRARY_TARGET)

ament_export_dependencies(
    common
    fastgrid
    geom
    map
    model
    nav_msgs
    OpenCV
)

install(
    TARGETS collision
    EXPORT collision
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")

    add_subdirectory("test")
endif()

ament_package()