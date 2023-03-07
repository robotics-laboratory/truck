#pragma once

#include <collision_checker/collision_checker.h>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <opencv2/core.hpp>

namespace truck::collision_checker {

cv::Mat distanceTransform(const cv::Mat& binary_map, double max_distance);

MapMeta toMapMetadata(const nav_msgs::msg::MapMetaData& map);

} // namespace truck::collision_checker