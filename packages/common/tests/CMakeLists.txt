ament_add_gtest(${PROJECT_NAME}_tests main.cpp)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

set_tests_properties(${Tests} PROPERTIES TIMEOUT 1)

target_include_directories(
  ${PROJECT_NAME}_tests
  PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
          "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
          "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
