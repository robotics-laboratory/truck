#pragma once

#include "icp_odometry/common.h"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


namespace truck::icp_odometry {
    std::optional <DataPoints> readNextDataPoints(rosbag2_cpp::Reader, rclcpp::Serialization <sensor_msgs::msg::LaserScan>);

    std::vector <DataPoints> readAllDataPoints(std::string);
}
