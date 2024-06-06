#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <pointmatcher/PointMatcher.h>

namespace truck::map_builder {

using DataPoints = PointMatcher<float>::DataPoints;

struct OdomWithPointCloud {
    nav_msgs::msg::Odometry odom;
    DataPoints data_points;
};

struct MapBuilderParams {};

class MapBuilder {
  public:
    MapBuilder(const MapBuilderParams& params, const std::vector<OdomWithPointCloud>& data);

    DataPoints build();

  private:
    MapBuilderParams params_;

    std::vector<OdomWithPointCloud> data_;
};

}  // namespace truck::map_builder