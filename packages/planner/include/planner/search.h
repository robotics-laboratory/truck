#pragma once

#include "geom/pose.h"
#include "geom/circle.h"
#include "collision/collision_checker.h"

#include <boost/geometry.hpp>

#include <optional>
#include <unordered_set>

namespace truck::planner::search {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using Point = bg::model::point<double, 2, bg::cs::cartesian>;
using Value = std::pair<Point, unsigned>;

struct Node {
    size_t index;
    geom::Vec2 point;
    bool finish;
    bool collision;
};

struct GridParams {
    int width;
    int height;
    double resolution;
    double finish_area_radius;
    double min_obstacle_distance;
};

class Grid {
  public:
    Grid(const GridParams& params, const model::Shape& shape);

    Grid& setEgoPose(const geom::Pose& ego_pose);
    Grid& setFinishArea(const geom::Circle& finish_area);
    Grid& setCollisionChecker(std::shared_ptr<const collision::StaticCollisionChecker> checker);
    Grid& build();

    const geom::Pose& getEgoPose() const;
    const std::vector<Node>& getNodes() const;
    const Node& getNodeByIndex(size_t index) const;
    const std::optional<size_t>& getEgoNodeIndex() const;
    const std::optional<size_t>& getFinishNodeIndex() const;
    const std::unordered_set<size_t>& getFinishAreaNodesIndices() const;

    bool insideFinishArea(const geom::Vec2& point) const;
    geom::Vec2 snapPoint(const geom::Vec2& point) const;

  private:
    GridParams params_;

    model::Shape shape_;

    geom::Pose ego_pose_;
    geom::Circle finish_area_;

    std::vector<Node> nodes_;

    static const size_t max_points_count = 16;
    bgi::rtree<Value, bgi::rstar<max_points_count>> nodes_points_;

    std::optional<size_t> ego_node_index_ = std::nullopt;
    std::optional<size_t> finish_node_index_ = std::nullopt;
    std::unordered_set<size_t> finish_area_nodes_indices_;

    std::shared_ptr<const collision::StaticCollisionChecker> checker_ = nullptr;
};

}  // namespace truck::planner::search