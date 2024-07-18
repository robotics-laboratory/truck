#pragma once

#include "lidar_map/common.h"

#include "geom/pose.h"
#include "geom/complex_polygon.h"

#include "rosbag2_cpp/reader.hpp"

namespace truck::lidar_map {

Clouds loadLidarScan(const std::string& mcap_path, const std::string& scan_topic);

geom::Poses loadOdometry(const std::string& mcap_path, const std::string& odom_topic);

void serializeToMCAP(
    const std::string& mcap_path, const Cloud& cloud, std::string cloud_topic = "/cloud",
    std::optional<geom::ComplexPolygon> map = std::nullopt, std::string map_topic = "/map");

}  // namespace truck::lidar_map
