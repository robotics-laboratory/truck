cmake_minimum_required(VERSION 3.8)
project(trajectory_planner)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter -Wno-inconsistent-missing-override)   

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(model REQUIRED)
find_package(collision REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED)

add_library(
    ${PROJECT_NAME}_lib SHARED 
    src/planner.cpp 
    src/rtree.cpp 
    src/state.cpp 
    src/tree.cpp 
    src/sampler.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib common geom model collision)

target_include_directories(
    ${PROJECT_NAME}_lib PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(common geom model collision)

install(
    TARGETS ${PROJECT_NAME}_lib
    EXPORT ${PROJECT_NAME}_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

add_executable(
    ${PROJECT_NAME}_node
    src/node.cpp
    src/main.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    common 
    geom 
    model 
    collision
    rclcpp
    tf2
    tf2_ros
    visualization_msgs
    Boost
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")

    add_subdirectory("test")
endif()

ament_package()
