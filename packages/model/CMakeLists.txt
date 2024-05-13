cmake_minimum_required(VERSION 3.5)
project(model)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pybind11 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(Boost REQUIRED)

set(CMAKE_CXX_STANDARD 20)

add_library(model SHARED src/params.cpp src/model.cpp)
ament_target_dependencies(
  model
  common
  geom
  rclcpp
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  yaml_cpp_vendor)

target_include_directories(model PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(modelTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  common
  geom
  rclcpp
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  yaml_cpp_vendor)

install(
  TARGETS model
  EXPORT modelTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

add_executable(model_tf_node src/model_tf.cpp)

ament_target_dependencies(
  model_tf_node
  rclcpp
  geometry_msgs
  tf2_msgs
  yaml_cpp_vendor)

target_link_libraries(model_tf_node model)

install(
    TARGETS model_tf_node
    DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(model_test test/main.cpp WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    target_link_libraries(model_test model)
endif()

set(PYBIND11_PYTHON_VERSION "3.8")
pybind11_add_module(pymodel ${CMAKE_SOURCE_DIR}/src/pybind.cpp)
target_link_libraries(pymodel PRIVATE model)

install(
  TARGETS pymodel
  LIBRARY DESTINATION lib/python3.8/site-packages)

ament_package()
