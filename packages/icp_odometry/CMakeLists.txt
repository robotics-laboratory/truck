cmake_minimum_required(VERSION 3.8)
project(icp_odometry)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

set(OpenGL_GL_PREFERENCE LEGACY)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
message("CMAKE_MODULE_PATH >>> " ${CMAKE_MODULE_PATH})

find_package(ament_cmake REQUIRED)
find_package(G2O REQUIRED)
message("G2O found >>> ${G2O_FOUND}")
message("G2O solvers found >>> ${G2O_SOLVERS_FOUND}")
message("G2O include >>> ${G2O_INCLUDE_DIR}")
set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
set(G2O_LIBRARIES ${G2O_SOLVER_EIGEN} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

include_directories(
  ${G2O_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/test_g2o.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  G2O
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${G2O_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
endif()

ament_package()
