#pragma once

#include "geom/pose.h"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <opencv2/core.hpp>

namespace truck::collision {

struct Map {
    static Map fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid);
    Map emptyLikeThis() const;

    nav_msgs::msg::OccupancyGrid makeCostMap(
        const std_msgs::msg::Header& header,
        double kMaxDist) const;

    geom::Pose origin;
    double resolution;
    cv::Size size;
    cv::Mat data;
};

Map distanceTransform(const Map& map);

}  // namespace truck::collision