#pragma once

#include "geom/pose.h"
#include "geom/circle.h"
#include "collision/collision_checker.h"

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <optional>

namespace truck::planner::search {

struct NodeId {
    int x, y;
    
    bool operator==(const NodeId& other) const {
      return (x == other.x) && (y == other.y);
    }
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

    Grid& setEgoPose(const std::optional<const geom::Pose>& ego_pose);
    Grid& setFinishArea(const std::optional<const geom::Circle>& finish_area);
    Grid& setCollisionChecker(std::shared_ptr<const collision::StaticCollisionChecker> checker);

    Grid& build();

    const std::vector<Node>& getNodes() const;
    const std::optional<geom::Pose>& getEgoPose() const;
    const std::optional<size_t>& getStartNodeIndex() const;
    const std::optional<size_t>& getEndNodeIndex() const;
    const std::set<size_t>& getFinishAreaNodesIndices() const;

    const Node& getNodeById(const NodeId& id) const;
    bool insideFinishArea(const geom::Vec2& point) const;
    geom::Vec2 snapPoint(const geom::Vec2& point) const;

    GridParams params;
    std::shared_ptr<const collision::StaticCollisionChecker> checker = nullptr;

  private:
    std::vector<Node> nodes_;
    std::set<size_t> finish_area_nodes_indices_;

    std::optional<size_t> start_node_index_ = std::nullopt;
    std::optional<size_t> end_node_index_ = std::nullopt;
    std::optional<geom::Pose> ego_pose_ = std::nullopt;
    std::optional<geom::Circle> finish_area_ = std::nullopt;
};

struct Primitive {
    NodeId shift_node_id;
    double length;
    std::vector<geom::Pose> poses;
};

class EdgeGeometryCache {
  public:
    EdgeGeometryCache();
    EdgeGeometryCache(const std::string& path);

    size_t getIndexByYaw(const geom::Angle& yaw) const;
    const std::vector<Primitive>& getPrimitives() const;

  private:
    std::vector<geom::Angle> yaws_;
    std::vector<Primitive> primitives_;
};

struct VertexSearchState {
    // exact cost of the path from the starting vertex to this vertex
    double start_cost = 0.0;

    // represents the heuristic estimated cost from this vertex to the finish vertex
    double heuristic_cost = 0.0;

    // lowest found cost (start_cost + heuristic_cost)
    double getTotalCost() const;

    // index of a previous vertex
    std::optional<size_t> prev_vertex_index = std::nullopt;

    // index of a primitive from previous to a current vertex
    std::optional<size_t> prev_to_cur_vertex_primitive_index = std::nullopt;
};

class DynamicGraph;

struct Vertex {
    NodeId node_id;
    size_t yaw_index;
    std::vector<size_t> edges_indices;

    VertexSearchState state;

    void updateState(const VertexSearchState& state);
};

class DynamicGraph {
  public:
    DynamicGraph();

    DynamicGraph& setGrid(std::shared_ptr<const Grid> grid);
    DynamicGraph& setEdgeGeometryCache(
        std::shared_ptr<const EdgeGeometryCache> edge_geometry_cache);

    bool checkConstraints(const Vertex* vertex, const Primitive& primitive) const;

    Vertex buildVertex(const NodeId& node_id, size_t yaw_index, const VertexSearchState& state);

    std::vector<Vertex> vertices;
    std::shared_ptr<const Grid> grid = nullptr;
    std::shared_ptr<const EdgeGeometryCache> edge_geometry_cache = nullptr;

  private:
    bool yawMask(const Primitive& primitive, const Vertex* vertex) const;
    bool boundaryMask(const Primitive& primitive, const Vertex* vertex) const;
    bool collisionMask(const Primitive& primitive, const Vertex* vertex) const;
};

class Searcher {
  public:
    Searcher();

    Searcher& setGraph(std::shared_ptr<DynamicGraph> graph);
    Searcher& findPath();

    void buildPath(size_t cur_vertex_index);

    const std::vector<geom::Vec2>& getPath() const;
    double getHeuristic(const geom::Vec2& from, const geom::Vec2& to) const;
    std::optional<size_t> getNeighborVertexIndex(const NodeId& node_id, size_t yaw_index) const;
    size_t findOptimalVertexIndex() const;

  private:
    std::vector<geom::Vec2> path_;
    std::set<size_t> open_set_, closed_set_;

    std::shared_ptr<DynamicGraph> graph_ = nullptr;
};

}  // namespace truck::planner::search