#pragma once

#include "geom/pose.h"
#include "geom/circle.h"
#include "collision/collision_checker.h"

#include <boost/geometry.hpp>

#include <optional>
#include <unordered_set>

namespace bg = boost::geometry;

using RTreePoint = bg::model::point<double, 2, bg::cs::cartesian>;
using RTreeIndexedPoint = std::pair<RTreePoint, size_t>;
using RTreeBox = bg::model::box<RTreePoint>;
using RTree = bg::index::rtree<RTreeIndexedPoint, bg::index::rstar<16>>;

namespace truck::planner::search {

struct Node {
    size_t index;
    geom::Vec2 point;

    bool ego;
    bool finish;
    bool finish_area;
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

  private:
    geom::Vec2 snapPoint(const geom::Vec2& point) const;

    void calculateNodes();
    void calculateEgoNode();
    void calculateFinishNode();
    void calculateFinishAreaNodes();

    struct NodeCache {
        RTree indexed_point_rtree;
        std::vector<Node> nodes;

        struct Index {
            std::optional<size_t> ego = std::nullopt;
            std::optional<size_t> finish = std::nullopt;
            std::unordered_set<size_t> finish_area;
        } index;
    } node_cache_;

    GridParams params_;

    model::Shape shape_;

    geom::Pose ego_pose_;
    geom::Circle finish_area_;

    std::shared_ptr<const collision::StaticCollisionChecker> checker_ = nullptr;
};

}  // namespace truck::planner::search