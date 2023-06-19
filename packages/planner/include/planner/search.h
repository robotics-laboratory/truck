#pragma once

#include "geom/pose.h"
#include "geom/circle.h"
#include "collision/collision_checker.h"

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <optional>

namespace truck::planner::search {

struct Color {
    double a, r, g, b;
};

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

    double node_z_lev;
    double node_scale;
    Color node_base_color;
    Color node_start_color;
    Color node_finish_base_color;
    Color node_finish_accent_color;
    Color node_collision_color;
};

struct GraphParams {
    double finish_area_radius;
    double min_obstacle_distance;

    double path_z_lev;
    double path_scale;
    Color path_color;
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
    geom::Vec2 clipPoint(const geom::Vec2& point) const;

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

struct Vertex;

class DynamicGraph;

class EdgeGeometryCache {
  public:
    EdgeGeometryCache();
    EdgeGeometryCache(const std::string& path);

    size_t getIndexByYaw(const geom::Angle& yaw) const;
    const std::vector<Primitive>& getPrimitives() const;

    bool checkConstraints(
        const Vertex* vertex,
        const Primitive& primitive,
        std::shared_ptr<const DynamicGraph> graph) const;

  private:
    bool boundaryMask(
      const Primitive& primitive,
      const NodeId& node_id,
      const GridParams& grid_params) const;

    bool collisionMask(
        const Primitive& primitive,
        double min_obstacle_distance,
        std::shared_ptr<const collision::StaticCollisionChecker> checker,
        const Node& node) const;

    bool yawMask(const Primitive& primitive, size_t yaw_index) const;

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

struct Vertex {
    Vertex(
        const NodeId& node_id,
        size_t yaw_index,
        const VertexSearchState& state,
        std::shared_ptr<const DynamicGraph> graph);

    NodeId node_id;
    size_t yaw_index;
    std::vector<size_t> edges_indices;

    VertexSearchState state;

    void update(const VertexSearchState& state);
};

class DynamicGraph {
  public:
    DynamicGraph(const GraphParams& params);

    DynamicGraph& setGrid(std::shared_ptr<const Grid> grid);
    DynamicGraph& setEdgeGeometryCache(
        std::shared_ptr<const EdgeGeometryCache> edge_geometry_cache);

    GraphParams params;
    std::vector<Vertex> vertices;
    std::shared_ptr<const Grid> grid = nullptr;
    std::shared_ptr<const EdgeGeometryCache> edge_geometry_cache = nullptr;
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

    std::shared_ptr<DynamicGraph> graph_ = nullptr;

  private:
    std::vector<geom::Vec2> path_;
    std::set<size_t> open_set_, closed_set_;
};

}  // namespace truck::planner::search