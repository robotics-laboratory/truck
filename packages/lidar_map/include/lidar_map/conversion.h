#pragma once

#include "lidar_map/common.h"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace truck::lidar_map {

Cloud toCloud(const sensor_msgs::msg::LaserScan& scan);

sensor_msgs::msg::PointCloud2 toPointCloud2(const Cloud& cloud, const std::string& frame_id);

}  // namespace truck::lidar_map
