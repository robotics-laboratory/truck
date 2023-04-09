#pragma once

#include "geom/pose.h"
#include "geom/circle.h"
#include "collision/collision_checker.h"

#include <set>
#include <memory>
#include <boost/property_tree/json_parser.hpp>

namespace truck::planner::search {

using Primitive = std::vector<geom::Pose>;

struct NodeId {
    int x;
    int y;
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
    float resolution;
};

class Grid {
  public:
    Grid(const std::vector<Node>& nodes);

    std::vector<Node> nodes_;
};

class GridBuilder {
  public:
    GridBuilder(const GridParams& params);

    GridBuilder& setEgoPose(const geom::Pose& ego_pose);
    GridBuilder& setFinishArea(const geom::Circle& finish_area);
    GridBuilder& setCollisionChecker(
        const std::shared_ptr<collision::StaticCollisionChecker> checker);

    Grid build();

  private:
    GridParams params_;

    std::optional<geom::Pose> ego_pose_ = std::nullopt;
    std::optional<geom::Circle> finish_area_ = std::nullopt;
    std::shared_ptr<collision::StaticCollisionChecker> checker_ = nullptr;

    bool insideFinishArea(const geom::Vec2& point);
};

class YawBins {
  public:
    YawBins(const std::string& path);

    std::vector<double> yaws_;
};

class EdgeGeometryCache {
  public:
    EdgeGeometryCache(const std::string& path);

    std::vector<Primitive> primitives_;
};

struct Vertex {
    Vertex();
    Vertex(
        NodeId node_id, size_t yaw_index,
        const std::shared_ptr<EdgeGeometryCache> edge_geometry_cache);

    NodeId node_id;
    size_t yaw_index;
    std::vector<Primitive*> edges;
};

class DynamicGraph {
  public:
    DynamicGraph();

    DynamicGraph& setGrid(const std::shared_ptr<Grid> grid);
    DynamicGraph& setYawBins(const std::shared_ptr<YawBins> yaw_bins);
    DynamicGraph& setEdgeGeometryCache(const std::shared_ptr<EdgeGeometryCache> edge_geometry_cache);
    std::vector<Vertex> vertices_;

    std::shared_ptr<Grid> grid_ = nullptr;
    std::shared_ptr<YawBins> yaw_bins_ = nullptr;
    std::shared_ptr<EdgeGeometryCache> edge_geometry_cache_ = nullptr;
};

class Searcher {
  public:
    Searcher(const std::shared_ptr<DynamicGraph> graph);

    void findPath();

    std::vector<geom::Pose> path_;
    std::shared_ptr<DynamicGraph> graph_ = nullptr;
};

}  // namespace truck::planner::search