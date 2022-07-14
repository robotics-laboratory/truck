cmake_minimum_required(VERSION 3.5)
project(model)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geom REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_library(model SHARED src/params.cpp src/model.cpp)

ament_target_dependencies(model common geom yaml-cpp)

target_include_directories(model PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(modelTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(common geom yaml-cpp)

install(
  TARGETS model
  EXPORT modelTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/model
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    #find_package(ament_cmake_clang_format REQUIRED)
    #find_file(CLANG_FORMAT .clang-format PATHS ${CMAKE_SOURCE_DIR}/../..)
    #ament_clang_format(CONFIG_FILE "${CLANG_FORMAT}" OTHER "--clang-format-version 10")

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(model_test test/main.cpp WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    target_link_libraries(model_test model yaml-cpp)
endif()

ament_package()
