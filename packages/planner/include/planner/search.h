#pragma once

#include "geom/pose.h"
#include "geom/circle.h"
#include "collision/collision_checker.h"

#include <rclcpp/rclcpp.hpp>

#include <optional>

namespace truck::planner::search {

struct NodeId {
    int x, y;

    bool operator==(const NodeId& other) const { return (x == other.x) && (y == other.y); }
};

struct Node {
    NodeId id;
    geom::Vec2 point;
    bool is_finish;
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
    Grid(const GridParams& params);

    Grid& setEgoPose(const geom::Pose& ego_pose);
    Grid& setFinishArea(const geom::Circle& finish_area);
    Grid& setCollisionChecker(std::shared_ptr<const collision::StaticCollisionChecker> checker);
    Grid& build();

    const geom::Pose& getEgoPose() const;
    const std::vector<Node>& getNodes() const;
    const Node& getNodeById(const NodeId& id) const;
    const std::optional<size_t>& getStartNodeIndex() const;
    const std::optional<size_t>& getEndNodeIndex() const;
    const std::unordered_set<size_t>& getFinishAreaNodesIndices() const;

    bool insideFinishArea(const geom::Vec2& point) const;
    geom::Vec2 snapPoint(const geom::Vec2& point) const;
    NodeId toNodeId(const geom::Vec2& point, const geom::Vec2& origin) const;

  private:
    GridParams params_;

    geom::Pose ego_pose_;
    geom::Circle finish_area_;

    std::vector<Node> nodes_;
    std::unordered_set<size_t> finish_area_nodes_indices_;

    std::optional<size_t> start_node_index_ = std::nullopt;
    std::optional<size_t> end_node_index_ = std::nullopt;

    std::shared_ptr<const collision::StaticCollisionChecker> checker_ = nullptr;
};

}  // namespace truck::planner::search