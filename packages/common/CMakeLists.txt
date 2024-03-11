cmake_minimum_required(VERSION 3.5)
project(common)

find_package(ament_cmake REQUIRED)

add_library(
    ${PROJECT_NAME} SHARED
    src/format.cpp
)

ament_target_dependencies(${PROJECT_NAME})

target_include_directories(
    ${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
# ament_export_dependencies(...)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory("tests")
endif()

ament_package()
