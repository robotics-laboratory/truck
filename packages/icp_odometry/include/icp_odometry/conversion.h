#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "icp_odometry/common.h"

namespace truck::icp_odometry {

DataPoints toDataPoints(const sensor_msgs::msg::LaserScan& scan);

sensor_msgs::msg::PointCloud2 toPointCloud2(const std_msgs::msg::Header& header, const DataPoints& data_points);

} // namespace truck::icp_odometry