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

    size_t getEgoNodeIndex() const;
    size_t getFinishNodeIndex() const;

    const geom::Pose& getEgoPose() const;
    const std::vector<Node>& getNodes() const;
    const Node& getNodeByIndex(size_t index) const;
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

    size_t ego_node_index_;
    size_t finish_node_index_;
    std::unordered_set<size_t> finish_area_nodes_indices_;

    std::shared_ptr<const collision::StaticCollisionChecker> checker_ = nullptr;
};

struct Edge {
    geom::Poses poses;
    double len;
    size_t end_pose_yaw_index;
};

using Edges = std::vector<Edge>;

struct EdgeInfo {
    size_t index;
    size_t end_pose_node_index;
    double len;
    size_t end_pose_yaw_index;
};

using EdgesInfo = std::vector<EdgeInfo>;

struct Vertex {
    size_t yaw_index;
    size_t node_index;

    struct SearchState {
        size_t prev_vertex_index;
        size_t prev_edge_index;
    } state;
};

class EdgeCache {
  public:
    virtual EdgesInfo getEdgesInfoFromVertex(const Vertex& vertex) = 0;
    virtual const geom::Poses& getPosesByEdgeIndex(size_t edge_index) = 0;

  private:
    size_t yaws_count;
    Edges edges;
    std::vector<std::vector<size_t>> yaws_to_edges_indexing;
};

}  // namespace truck::planner::search