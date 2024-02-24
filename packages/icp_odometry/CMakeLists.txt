cmake_minimum_required(VERSION 3.8)
project(icp_odometry)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(Boost COMPONENTS thread filesystem system program_options date_time chrono timer serialization REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rosbag2_cpp REQUIRED)
#find_package(rosbag2_storage REQUIRED)
#find_package(visualization_msgs REQUIRED)

find_package(g2o REQUIRED) # G2O FIND PACKAGE

set(CMAKE_CXX_STANDARD 20)

add_executable(
  ${PROJECT_NAME}_node
#  src/main.cpp
  src/icp_odometry_node.cpp
  src/conversion.cpp
#  src/import_bag.cpp

  src/test_g2o.cpp # G2O EXEC
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    common
    geom
    libpointmatcher
    rclcpp
    sensor_msgs
    Boost
    truck_msgs
#    rosbag2_cpp
#    rosbag2_storage
#    visualization_msgs

    g2o # G2O TARGET DEP
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME})

install(
#    DIRECTORY launch config bags
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
endif()

ament_package()
