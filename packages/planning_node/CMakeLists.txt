cmake_minimum_required(VERSION 3.8)
project(planning_node)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(planning_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

include(FetchContent)

FetchContent_Declare(
  float_comparison
  GIT_REPOSITORY https://github.com/packedbread/float_comparison
  GIT_TAG master
)

FetchContent_MakeAvailable(float_comparison)

# FetchContent_Declare(
#   json
#   URL https://github.com/nlohmann/json/releases/download/v3.10.5/json.tar.xz
#   URL_HASH SHA256=344be97b757a36c5b180f1c8162f6c5f6ebd760b117f6e64b77866e97b217280
# )

# FetchContent_GetProperties(json)
# if(NOT json_POPULATED)
#   FetchContent_Populate(json)
#   add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR} EXCLUDE_FROM_ALL)
# endif()

add_executable(node
  src/main.cpp
  src/node.cpp
  src/node.hpp
  src/planner.cpp
  src/planner.hpp
  src/single_slot_queue.hpp
  thirdparty/json.hpp
)
target_include_directories(node PUBLIC ${float_comparison_SOURCE_DIR} ${PROJECT_SOURCE_DIR})
# target_link_libraries(node nlohmann_json::nlohmann_json)
target_compile_features(node PUBLIC c_std_11 cxx_std_20)
ament_target_dependencies(node planning_interfaces rclcpp)

install(TARGETS
  node
  DESTINATION lib/${PROJECT_NAME}
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
