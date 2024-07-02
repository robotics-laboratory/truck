#pragma once

#include "lidar_map/common.h"

#include "geom/complex_polygon.h"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace truck::lidar_map {

Cloud toCloud(const sensor_msgs::msg::LaserScan& scan, double z_lev = 1.0);

sensor_msgs::msg::PointCloud2 toPointCloud2(const Cloud& cloud, std::string frame_id = "");

visualization_msgs::msg::Marker toMarker(
    const geom::ComplexPolygon& complex_polygon, std::string frame_id = "", double z_lev = 1.0,
    std::vector<double> rgba_color = {0.4, 0.4, 0.4, 1.0});

}  // namespace truck::lidar_map
