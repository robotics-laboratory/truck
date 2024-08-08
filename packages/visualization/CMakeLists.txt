cmake_minimum_required(VERSION 3.5)
project(visualization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(map REQUIRED)
find_package(geom REQUIRED)
find_package(model REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} SHARED src/color.cpp src/msg.cpp)

ament_target_dependencies(
  ${PROJECT_NAME} geom visualization_msgs std_msgs truck_msgs Boost
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(geom visualization_msgs std_msgs truck_msgs Boost)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node src/main.cpp src/visualization_node.cpp)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  map
  model
  nav_msgs
  truck_msgs
  visualization_msgs
  tf2_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
