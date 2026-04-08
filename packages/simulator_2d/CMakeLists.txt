cmake_minimum_required(VERSION 3.5)
project(simulator_2d)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(truck_msgs REQUIRED)
find_package(geom REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(model REQUIRED)
find_package(map REQUIRED)
find_package(simulator2d_core REQUIRED)


add_executable(${PROJECT_NAME}_node
    src/truck_simulator_node.cpp
    src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    truck_msgs
    geom
    simulator2d_core
)

target_link_libraries(${PROJECT_NAME}_node
    /home/andrew/sim2d/sim2d/install/simulator2d_core/lib/libsimulator2d_core.so
)

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()