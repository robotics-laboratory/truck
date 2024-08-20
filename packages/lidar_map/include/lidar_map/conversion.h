#pragma once

#include "lidar_map/common.h"
#include "geom/msg.h"
#include "geom/pose.h"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace truck::lidar_map {

geom::Poses toPoses(const std::vector<nav_msgs::msg::Odometry>& odom_msgs);

Cloud toCloud(const sensor_msgs::msg::LaserScan& scan);

Clouds toClouds(const std::vector<sensor_msgs::msg::LaserScan>& scans);

namespace msg {

sensor_msgs::msg::PointCloud2 toPointCloud2(const Cloud& cloud, std::string frame_id = "world");

}  // namespace msg

}  // namespace truck::lidar_map
