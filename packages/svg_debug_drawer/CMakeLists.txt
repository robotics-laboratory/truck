cmake_minimum_required(VERSION 3.8)
project(svg_debug_drawer)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)   

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(pugixml REQUIRED)

add_library(
    ${PROJECT_NAME} SHARED
    src/sdd.cpp
)

ament_target_dependencies(${PROJECT_NAME} common geom pugixml)

target_link_libraries(${PROJECT_NAME} pugixml)

target_include_directories(
    ${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(common geom pugixml)

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
