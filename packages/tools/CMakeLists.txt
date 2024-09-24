cmake_minimum_required(VERSION 3.8)
project(tools)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED src/progress_bar.cpp src/benchmark.cpp)

ament_target_dependencies(${PROJECT_NAME})

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
