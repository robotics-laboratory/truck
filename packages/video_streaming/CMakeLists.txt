cmake_minimum_required(VERSION 3.8)
project(video_streaming)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES server.js package.json DESTINATION share/${PROJECT_NAME})
execute_process(
  COMMAND "npm" "install" "--no-audit" "--no-fund"
  WORKING_DIRECTORY /truck/packages/install/share/${PROJECT_NAME}
  OUTPUT_VARIABLE CMD_OUTPUT ERROR_VARIABLE CMD_OUTPUT
)
message(WARNING "${CMD_OUTPUT}")

ament_package()
