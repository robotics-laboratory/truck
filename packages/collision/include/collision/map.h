#pragma once

#include "geom/complex_polygon.h"
#include "geom/pose.h"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"

#include "map/map_builder.h"

#include <opencv2/core.hpp>

namespace truck::collision {

class CollisionMap {
  public:
    CollisionMap& SetOccupancyGrid(
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid) noexcept;
    CollisionMap& SetMap(std::shared_ptr<const map::Map> map) noexcept;

    fastgrid::U8Grid GetBitMap() noexcept;
    fastgrid::F32Grid GetDistanceMap() noexcept;

    nav_msgs::msg::OccupancyGrid MakeCostMap(
        const std_msgs::msg::Header& header, double kMaxDist) noexcept;

  private:
    struct State {
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid = nullptr;
        std::shared_ptr<const map::Map> map = nullptr;
    } state_;

    struct BitMap {
        bool is_consistent = true;
        fastgrid::U8GridHolder holder;
    } bit_map_;

    struct DistanceMap {
        bool is_consistent = true;
        fastgrid::F32GridHolder holder;
        fastgrid::S32GridHolder buffer;
    } distance_map_;
};

}  // namespace truck::collision