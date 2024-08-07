cmake_minimum_required(VERSION 3.5)
project(simulator_2d)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(model REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(geom REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(map REQUIRED)
find_package(Boost REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(
  simulator_engine SHARED src/simulator_engine.cpp src/truck_state.cpp
                          src/simulation_map.cpp
)

ament_target_dependencies(simulator_engine model geom map tf2 Boost)

target_include_directories(
  simulator_engine
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(simulator_engine_targets HAS_LIBRARY_TARGET)

ament_export_dependencies(model geom nlohmann_json)

install(
  TARGETS simulator_engine
  EXPORT simulator_engine_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node src/simulator_node.cpp src/main.cpp)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  model
  nav_msgs
  truck_msgs
  geom
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  rosgraph_msgs
  sensor_msgs
  nlohmann_json
)

target_link_libraries(${PROJECT_NAME}_node simulator_engine)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    ${PROJECT_NAME}_tests test/main.cpp test/simulation_map_tests.cpp
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  ament_target_dependencies(
    ${PROJECT_NAME}_tests
    model
    truck_msgs
    geom
    rclcpp
    rosbag2_cpp
    rosbag2_storage
    ament_index_cpp
  )

  target_link_libraries(${PROJECT_NAME}_tests simulator_engine)
endif()

ament_package()
