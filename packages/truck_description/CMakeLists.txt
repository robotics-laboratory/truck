cmake_minimum_required(VERSION 3.8)
project(truck_description)

find_package(ament_cmake REQUIRED)

install(FILES truck.sdf model.config DESTINATION share/${PROJECT_NAME})

ament_package()
