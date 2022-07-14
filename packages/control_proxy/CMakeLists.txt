cmake_minimum_required(VERSION 3.5)
project(control_proxy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(model REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(truck_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_executable(control_proxy src/main.cpp src/control_proxy.cpp)

ament_target_dependencies(control_proxy model rclcpp sensor_msgs truck_interfaces yaml-cpp)
target_link_libraries(control_proxy yaml-cpp)

install(
    TARGETS control_proxy
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

#if(BUILD_TESTING)
#    find_package(ament_cmake_clang_format REQUIRED)
#    find_file(CLANG_FORMAT .clang-format PATHS ${CMAKE_SOURCE_DIR}/../..)
#    ament_clang_format(CONFIG_FILE "${CLANG_FORMAT}" OTHER "--clang-format-version 10")

#    find_package(ament_cmake_gtest REQUIRED)
#    ament_add_gtest(model_test test/main.cpp WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
#    target_link_libraries(model_test model yaml-cpp)
#endif()

ament_package()
