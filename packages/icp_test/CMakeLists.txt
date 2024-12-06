cmake_minimum_required(VERSION 3.8)
project(icp_test)


find_package(ament_cmake REQUIRED)

find_package(geom REQUIRED)
find_package(common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(map REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(G2O REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem system program_options date_time chrono)

include_directories(include ${PCL_INCLUDE_DIRS} ${rosbag2_cpp_INCLUDE_DIRS} ${libpointmatcher_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS})




add_executable(main_executable src/main.cpp)

ament_target_dependencies(
    main_executable
    rclcpp
    rosbag2_cpp
    sensor_msgs
)


target_link_libraries(main_executable ${PCL_LIBRARIES} ${libpointmatcher_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS main_executable DESTINATION lib/${PROJECT_NAME})

ament_package()