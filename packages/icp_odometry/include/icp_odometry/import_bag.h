#pragma once

#include "icp_odometry/common.h"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>


namespace truck::icp_odometry {
	struct ICPOdometryData {
		DataPoints icpDataPoints;
		nav_msgs::msg::Odometry odometry;
		nav_msgs::msg::Odometry optimizedOdometry;
	};

	std::vector<ICPOdometryData> readAllICPOdometryData(std::string, size_t = -1);
	std::vector<DataPoints> readAllDataPoints(std::string);
}
