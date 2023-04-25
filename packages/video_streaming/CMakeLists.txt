cmake_minimum_required(VERSION 3.8)
project(video_streaming)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES mediamtx.yaml DESTINATION share/${PROJECT_NAME})

ament_package()
