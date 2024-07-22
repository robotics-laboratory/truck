ament_add_gtest(
  ${PROJECT_NAME}_tests main.cpp WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}_tests map ament_index_cpp)

target_include_directories(
  ${PROJECT_NAME}_tests
  PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
          "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
          "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
