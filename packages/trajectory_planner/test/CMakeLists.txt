find_package(ament_cmake_gtest REQUIRED)

find_package(svg_debug_drawer REQUIRED)
find_package(map REQUIRED)
find_package(fmt REQUIRED)

ament_add_gtest(${PROJECT_NAME}_tests main.cpp)

ament_target_dependencies(${PROJECT_NAME}_tests svg_debug_drawer map fmt)

ament_export_dependencies(svg_debug_drawer map fmt)

target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME}_lib fmt)
set_tests_properties(${Tests} PROPERTIES TIMEOUT 10)

target_include_directories(${PROJECT_NAME}_tests PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
