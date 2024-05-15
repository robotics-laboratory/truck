cmake_minimum_required(VERSION 3.5)
project(speed)

find_package(ament_cmake REQUIRED)
find_package(model REQUIRED)
find_package(motion REQUIRED)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} SHARED src/greedy_planner.cpp)

ament_target_dependencies(${PROJECT_NAME} common model motion Boost)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost model motion)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
