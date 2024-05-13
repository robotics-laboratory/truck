find_package(ament_cmake_google_benchmark REQUIRED)
find_package(OpenCV REQUIRED)

ament_add_google_benchmark(${PROJECT_NAME}_benchmarks main.cpp TIMEOUT 100)
target_link_libraries(${PROJECT_NAME}_benchmarks ${PROJECT_NAME} ${OpenCV_LIBS})

target_include_directories(
  ${PROJECT_NAME}_benchmarks
  PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
          "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
          "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
