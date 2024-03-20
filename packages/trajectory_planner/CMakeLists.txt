cmake_minimum_required(VERSION 3.8)
project(trajectory_planner)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter -Wno-inconsistent-missing-override)   

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(motion REQUIRED)

add_library(
    ${PROJECT_NAME} SHARED src/state.cpp
)

ament_target_dependencies(${PROJECT_NAME} common geom motion)

target_include_directories(
    ${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    "$<INSTALL_INTERFACE:include/hnswlib>"
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(common geom motion)

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
endif()

ament_package()
