#pragma once

#include "geom/pose.h"
#include "geom/square.h"
#include "collision/collision_checker.h"

#include <nlohmann/json.hpp>
#include <boost/geometry.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <fstream>
#include <optional>
#include <algorithm>
#include <unordered_set>

namespace truck::planner::search {

namespace bg = boost::geometry;

using Point = bg::model::point<double, 2, bg::cs::cartesian>;
using IndexedPoint = std::pair<Point, unsigned>;
using Box = bg::model::box<Point>;
using RTree = bg::index::rtree<IndexedPoint, bg::index::rstar<16>>;

struct Node {
    size_t index;
    geom::Vec2 point;
    bool collision;
};

struct GridParams {
    size_t width;
    size_t height;
    double resolution;
};

class Grid {
  public:
    Grid(const GridParams& params, const model::Shape& shape);

    Grid& setEgoPose(const geom::Pose& ego_pose);
    Grid& setFinishArea(const geom::Square& finish_area);
    Grid& setCollisionChecker(std::shared_ptr<const collision::StaticCollisionChecker> checker);

    const geom::Pose& getEgoPose() const;

    const std::vector<Node>& getNodes() const;

    const Node& getNodeByIndex(size_t index) const;

    size_t getNodeIndexByPoint(const geom::Vec2& point) const;

    size_t getEgoNodeIndex() const;
    size_t getFinishNodeIndex() const;
    const std::unordered_set<size_t>& getFinishAreaNodesIndices() const;

    geom::Vec2 snapPoint(const geom::Vec2& point) const;

    void calculateNodesIndices();

    bool finishPointInsideBorders(const geom::Vec2& point);

    bool build();

  private:
    GridParams params_;

    model::Shape shape_;

    geom::Pose ego_pose_;
    geom::Square finish_area_;

    struct NodeCache {
        RTree rtree;
        std::vector<Node> nodes;

        size_t ego_index;
        size_t finish_index;
        std::unordered_set<size_t> finish_area_indices;
    } cache_;

    std::shared_ptr<const collision::StaticCollisionChecker> checker_ = nullptr;
};

struct Vertex {
    size_t yaw_index;
    size_t node_index;

    struct SearchState {
        // exact cost of the path from the starting vertex to this vertex
        double start_cost = 0.0;

        // represents the heuristic estimated cost from this vertex to the finish vertex
        double heuristic_cost = 0.0;

        // index of a previous vertex
        std::optional<size_t> prev_vertex_index;

        // index of a previous edge
        std::optional<size_t> prev_edge_index;

        // lowest found cost: start_cost + heuristic_cost
        double getTotalCost() const;
    } state;
};

struct EdgeParams {
    size_t yaws_count;
    std::string type;

    struct PrimitiveParams {
        std::string json_path;
    } primitive;
};

struct Edge {
    geom::Poses poses;

    size_t finish_yaw_index;
    double len;
};

struct EdgeInfo {
    size_t index;
    geom::Vec2 finish_point;

    size_t finish_yaw_index;
    double len;
};

using Edges = std::vector<Edge>;
using EdgesInfo = std::vector<EdgeInfo>;

class EdgeCache {
  public:
    size_t getYawsCount() const;

    size_t getYawIndexFromAngle(double theta) const;

    const geom::Poses& getPosesByEdgeIndex(size_t index) const;

    virtual EdgesInfo getEdgesInfoByYawIndex(size_t yaw_index) const = 0;

  protected:
    EdgeParams edge_params_;
    Edges edges_;

    // Storing indices of all edges.
    // The 'i'-th cell of this array stores the array of edges indexes
    // that start from the angle corresponding to yaw with the 'i'-th index.
    std::vector<std::vector<size_t>> edges_indices_;

  private:
    // Storing the full angle value of 360 degrees in radial equivalent.
    // Used to find the nearest yaw index to an arbitrary angle rad in interval [-PI; PI)
    const double full_angle = 6.28;
};

class PrimitiveCache : public EdgeCache {
  public:
    PrimitiveCache(const EdgeParams& edge_params);

    EdgesInfo getEdgesInfoByYawIndex(size_t yaw_index) const override;

  private:
    void parseJSON();
};

struct SearcherParams {
    int max_vertices_count;
    double finish_area_size;
    double min_obstacle_distance;
    double max_node_position_error;
};

class Searcher {
  public:
    Searcher();

    void setParams(const SearcherParams& params);
    void setGrid(std::shared_ptr<const Grid> grid);
    void setEdgeCache(std::shared_ptr<const EdgeCache> edge_cache);
    void setCollisionChecker(std::shared_ptr<const collision::StaticCollisionChecker> checker);

    bool findPath();
    geom::Poses getPath() const;

    void reset();

  private:
    void buildState();

    void resetVertices();

    void addVertex(const Vertex& vertex);

    size_t getVertexIndexFromOpenSet() const;
    std::optional<size_t> getVertexIndex(size_t yaw_index, size_t node_index) const;

    bool collisionFree(const EdgeInfo& edge) const;
    bool insideBorders(const EdgeInfo& edge) const;
    double calculateHeuristic(size_t node_index) const;

    SearcherParams params_;

    struct Stopwatch {
        void start();
        void end();

        std::chrono::high_resolution_clock::time_point t1, t2;
    } stopwatch_;

    struct State {
        std::vector<geom::Poses> edges;
        RTree rtree_poses;

        bool empty() const;

        void clear();
        void reverse();

        void addEdge(const geom::Poses& edge, const geom::Vec2& origin);
        void addEdgeFromPrevState(const geom::Poses& edge);

        size_t getNearestEdgeIndexByEgo(const geom::Pose& ego) const;
        geom::Pose getStartPoseByEgo(const geom::Pose& ego) const;
        geom::Poses constructPath(const geom::Pose& ego) const;
    } current_state_, prev_state_;

    geom::Pose start_pose;

    size_t current_vertex_index_;
    std::vector<Vertex> vertices_;
    std::vector<std::vector<size_t>> vertices_indices_;
    std::unordered_set<size_t> open_set_, closed_set_;

    std::shared_ptr<const Grid> grid_ = nullptr;
    std::shared_ptr<const EdgeCache> edge_cache_ = nullptr;
    std::shared_ptr<const collision::StaticCollisionChecker> checker_ = nullptr;
};

}  // namespace truck::planner::search