cmake_minimum_required(VERSION 3.8)
project(map)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(geom REQUIRED)
find_package(common REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(
    ${PROJECT_NAME} SHARED
    src/map.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}
    geom
    common
    nlohmann_json
)

target_include_directories(
    ${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
    geom
    common
    nlohmann_json
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

install(
    DIRECTORY data
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
