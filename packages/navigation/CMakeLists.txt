cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(map REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(${PROJECT_NAME}_mesh SHARED src/mesh_builder.cpp)

add_library(${PROJECT_NAME}_graph SHARED src/graph_builder.cpp src/search.cpp)

add_library(${PROJECT_NAME}_viewer SHARED src/viewer.cpp)

ament_target_dependencies(${PROJECT_NAME}_mesh common geom map CGAL Boost)

ament_target_dependencies(${PROJECT_NAME}_graph common geom Boost)

ament_target_dependencies(${PROJECT_NAME}_viewer common geom OpenCV)

target_include_directories(
  ${PROJECT_NAME}_mesh
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_include_directories(
  ${PROJECT_NAME}_graph
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_include_directories(
  ${PROJECT_NAME}_viewer
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)

ament_export_dependencies(common geom map CGAL Boost OpenCV)

install(
  TARGETS ${PROJECT_NAME}_mesh ${PROJECT_NAME}_graph ${PROJECT_NAME}_viewer
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    ${PROJECT_NAME}_tests test/main.cpp WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  ament_target_dependencies(${PROJECT_NAME}_tests ament_index_cpp)

  target_link_libraries(
    ${PROJECT_NAME}_tests ${PROJECT_NAME}_mesh ${PROJECT_NAME}_graph
    ${PROJECT_NAME}_viewer
  )
endif()

ament_package()
