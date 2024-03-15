find_package(ament_cmake_google_benchmark REQUIRED)

include(FetchContent)
FetchContent_Declare(
  hnswlib  
  GIT_REPOSITORY https://github.com/nmslib/hnswlib.git
  GIT_TAG "origin/master"
)
FetchContent_MakeAvailable(hnswlib)

ament_add_google_benchmark(${PROJECT_NAME}_benchmarks main.cpp TIMEOUT 100)
target_link_libraries(${PROJECT_NAME}_benchmarks ${PROJECT_NAME} hnswlib)

target_include_directories(${PROJECT_NAME}_benchmarks PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "$<INSTALL_INTERFACE:include/hnswlib>"
)
