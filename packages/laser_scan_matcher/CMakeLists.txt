cmake_minimum_required(VERSION 3.8)
project(laser_scan_matcher)

set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
#colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=DEBUG --cmake-args -DCMAKE_C_FLAGS_DEBUG="-g -O0" --cmake-args -DCMAKE_CXX_FLAGS_DEBUG="-g -O0"

find_package(ament_cmake REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosbag2_storage REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(laser_scan_matcher src/main.cpp)
ament_target_dependencies(laser_scan_matcher pcl_conversions tf2_ros tf2 nav_msgs rclcpp rosbag2_cpp common_interfaces laser_geometry rosbag2_storage)

target_link_libraries(laser_scan_matcher ${PCL_LIBRARIES} 
  ${rosbag2_cpp_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${common_interfaces_LIBRARIES}
  ${rosbag2_storage_LIBRARIES}
  ${laser_geometry_LIBRARIES})

target_include_directories(laser_scan_matcher PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "${ROS_ROOT}/share"
  "${rclcpp_INCLUDE_DIRS}"
  "${common_interfaces_INCLUDE_DIRS}")

install(TARGETS laser_scan_matcher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
