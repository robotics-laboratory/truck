find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(model REQUIRED)
find_package(truck_msgs REQUIRED)

ament_add_gtest (
    ${PROJECT_NAME}_test
    main.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_test
    model
    truck_msgs
    geom
    rclcpp
    rosbag2_cpp
    rosbag2_storage
)

target_link_libraries(
    ${PROJECT_NAME}_test 
    simulator_engine
)
