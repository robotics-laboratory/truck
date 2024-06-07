cmake_minimum_required(VERSION 3.8)
project(inf-error)

find_package(ament_cmake REQUIRED)

add_executable(
  ${PROJECT_NAME}_node src/main.cpp
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

ament_package()
