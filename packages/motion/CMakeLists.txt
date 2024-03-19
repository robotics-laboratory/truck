cmake_minimum_required(VERSION 3.5)
project(motion)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(model REQUIRED)
find_package(truck_msgs REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/trajectory.cpp
  src/primitive.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  common
  geom
  model
  truck_msgs)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common model truck_msgs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
