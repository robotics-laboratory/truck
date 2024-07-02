cmake_minimum_required(VERSION 3.8)
project(lidar_map)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geom REQUIRED)
find_package(common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(
  Boost
  COMPONENTS thread
             filesystem
             system
             program_options
             date_time
             chrono
             timer
             serialization
             REQUIRED
)

add_library(
  ${PROJECT_NAME} SHARED src/builder.cpp src/serializer.cpp src/conversion.cpp
                         src/icp.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  geom
  common
  nav_msgs
  sensor_msgs
  rosbag2_cpp
  libpointmatcher
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  geom
  common
  nav_msgs
  sensor_msgs
  rosbag2_cpp
  libpointmatcher
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  # find_package(ament_cmake_gtest REQUIRED) add_subdirectory("test")
endif()

ament_package()
