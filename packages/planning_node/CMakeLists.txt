cmake_minimum_required(VERSION 3.8)
project(planning_node)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(planning_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

include(FetchContent)

FetchContent_Declare(
  float_comparison
  GIT_REPOSITORY https://github.com/packedbread/float_comparison
  GIT_TAG master
)

FetchContent_MakeAvailable(float_comparison)

add_executable(
  planning_node
  src/main.cpp
  src/node.cpp
  src/node.hpp
  src/planner.cpp
  src/planner.hpp
  src/shared_state.hpp
  src/geometry.hpp
)
target_include_directories(
  planning_node
  PUBLIC
  ${float_comparison_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)
target_link_libraries(planning_node ${OpenCV_LIBS})
target_compile_features(planning_node PUBLIC c_std_11 cxx_std_17)
ament_target_dependencies(planning_node planning_interfaces rclcpp)

install(
  TARGETS
  planning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  DESTINATION
  share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
