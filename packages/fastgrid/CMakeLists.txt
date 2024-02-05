cmake_minimum_required(VERSION 3.8)
project(fastgrid)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)   

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(OpenCV REQUIRED)

add_library(
    ${PROJECT_NAME} SHARED
    src/distance_transform.cpp
    src/grid.cpp
    src/manhattan_distance.cpp
    src/draw.cpp
)

ament_target_dependencies(${PROJECT_NAME} common geom OpenCV)

target_include_directories(
    ${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(common geom OpenCV)

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

    add_subdirectory("test")
    # add_subdirectory("benchmark")
endif()

ament_package()
