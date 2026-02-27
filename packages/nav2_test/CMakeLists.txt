cmake_minimum_required(VERSION 3.8)
project(nav2_test)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS proxy.py DESTINATION lib/${PROJECT_NAME})

ament_package()
