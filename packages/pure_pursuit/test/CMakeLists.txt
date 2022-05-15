ament_add_gtest(${PROJECT_NAME}_tests main.cpp
    ENV TEST_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/../../model/config/model.yaml)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME}_simulator)

set_tests_properties(${Tests} PROPERTIES TIMEOUT 1)

target_include_directories(${PROJECT_NAME}_tests PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
