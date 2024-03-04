cmake_minimum_required(VERSION 3.8)
project(hardware_node_2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(model REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp src/hardware_node_2.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs model truck_msgs nav_msgs tf2_geometry_msgs)

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    # "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )

  
target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")
    
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()