cmake_minimum_required(VERSION 3.8)
project(truck_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

xacro_add_xacro_file(
   ${CMAKE_CURRENT_SOURCE_DIR}/urdf/truck.xacro
   ${CMAKE_CURRENT_SOURCE_DIR}/urdf/truck.urdf)

add_custom_target(generate_urdfs ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/truck.urdf)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/urdf/truck.urdf DESTINATION share/${PROJECT_NAME}/urdf)
install(FILES model.config DESTINATION share/${PROJECT_NAME})

install(
   DIRECTORY config urdf
   DESTINATION share/${PROJECT_NAME})

ament_package()
