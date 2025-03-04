cmake_minimum_required(VERSION 3.8)
project(motion_planner)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(map REQUIRED)
find_package(speed REQUIRED)
find_package(collision REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(
  ${PROJECT_NAME}_node src/main.cpp src/motion_planner_node.cpp
                       src/motion_planner.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  truck_msgs
  nav_msgs
  geometry_msgs
  common
  geom
  map
  speed
  collision
  tf2
  tf2_ros
  std_srvs
  visualization_msgs
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

target_include_directories(
  ${PROJECT_NAME}_node
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    ${PROJECT_NAME}_tests test/main.cpp WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
  ament_target_dependencies(${PROJECT_NAME}_tests ament_index_cpp)
endif()

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
