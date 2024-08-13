#pragma once

#include "lidar_map/common.h"
#include "geom/complex_polygon.h"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace truck::lidar_map {

std::vector<nav_msgs::msg::Odometry> loadOdomTopic(
    const std::string& mcap_path, const std::string& odom_topic);

std::vector<sensor_msgs::msg::LaserScan> loadLaserScanTopic(
    const std::string& mcap_path, const std::string& laser_scan_topic);

void syncOdomWithCloud(
    std::vector<nav_msgs::msg::Odometry>& odom_msgs,
    std::vector<sensor_msgs::msg::LaserScan>& laser_scan_msgs);

void writeToMCAP(
    const std::string& mcap_path, const Cloud& cloud, const std::string& cloud_topic_name);

void writeToMCAP(
    const std::string& mcap_path, const Cloud& cloud, const std::string& cloud_topic_name,
    const geom::ComplexPolygon& map, const std::string& map_topic_name);

void writeToPCD(const std::string& pcd_path, const Cloud& cloud);

}  // namespace truck::lidar_map
