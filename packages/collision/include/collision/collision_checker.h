#pragma once

#include "geom/pose.h"
#include "geom/transform.h"
#include "geom/vector.h"

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"

#include "map/map.h"

#include "model/params.h"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <memory>
#include <optional>

namespace truck::collision {

class StaticCollisionChecker {
  public:
    constexpr static double kMaxDistance = 10.0;

    struct Params {
        double radius = 20;
        double resolution = 0.1;
    };

    StaticCollisionChecker(const Params& params, const model::Shape& shape);

    bool Initialized() const noexcept;

    StaticCollisionChecker& SetEgoPose(const geom::Pose& ego_pose) noexcept;

    StaticCollisionChecker& SetOccupancyGrid(
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid) noexcept;

    StaticCollisionChecker& SetMap(std::shared_ptr<const map::Map> map) noexcept;

    fastgrid::U8Grid BitMap() const noexcept;

    fastgrid::F32Grid DistanceMap() const noexcept;

    nav_msgs::msg::OccupancyGrid MakeCostMap(
        const std_msgs::msg::Header& header, double kMaxDist = kMaxDistance) const noexcept;

    double Distance(const geom::Pose& ego_pose) const noexcept;

    double Distance(const geom::Vec2& point) const noexcept;

  private:
    void ResetCache() noexcept;

    Params params_;
    model::Shape shape_;

    mutable struct BitMapCache {
        fastgrid::U8GridHolder bit_map;
        bool is_consistent = true;
    } bit_map_cache_;

    mutable struct DistanceMapCache {
        fastgrid::S32GridHolder buffer;
        fastgrid::F32GridHolder distance_map;
        bool is_consistent = true;
    } distance_map_cache_;

    struct State {
        std::optional<geom::Pose> origin = std::nullopt;
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid = nullptr;
        std::shared_ptr<const map::Map> map = nullptr;
    } state_;
};

}  // namespace truck::collision
