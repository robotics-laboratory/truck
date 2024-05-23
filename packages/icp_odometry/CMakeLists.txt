cmake_minimum_required(VERSION 3.8)
project(icp_odometry)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

find_package(ament_cmake REQUIRED)
find_package(Boost COMPONENTS thread filesystem system program_options date_time chrono timer serialization REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(g2o REQUIRED) # G2O FIND PACKAGE
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(OpenGL_GL_PREFERENCE LEGACY)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
message("CMAKE_MODULE_PATH >>> " ${CMAKE_MODULE_PATH})

set(CMAKE_CXX_STANDARD 20)
find_package(ament_cmake REQUIRED)
find_package(G2O REQUIRED)
message("G2O found >>> ${G2O_FOUND}")
message("G2O solvers found >>> ${G2O_SOLVERS_FOUND}")
message("G2O include >>> ${G2O_INCLUDE_DIR}")
set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
set(G2O_LIBRARIES ${G2O_SOLVER_EIGEN} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_TYPES_SLAM2D})

include_directories(
        ${G2O_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/main.cpp
  src/g2o.cpp
  src/icp.cpp
  src/import_bag.cpp
  src/icp_odometry_node.cpp
  src/conversion.cpp
  src/visualization.cpp
  src/config.cpp
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
    rosbag2_cpp
    rosbag2_storage
    visualization_msgs
    g2o
    tf2_geometry_msgs
    yaml-cpp
)

target_link_libraries(
    ${PROJECT_NAME}_node
    ${G2O_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch config bags
    DESTINATION share/${PROJECT_NAME})

ament_package()
