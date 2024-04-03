#pragma once

#include "geom/pose.h"
#include "geom/circle.h"
#include "geom/polyline.h"
#include "collision/collision_checker.h"

#include <boost/geometry.hpp>

#include <set>
#include <unordered_set>
#include <unordered_map>
#include <optional>

namespace truck::routing_planner::search {

namespace bg = boost::geometry;

using RTreePoint = bg::model::point<double, 2, bg::cs::cartesian>;
using RTreeIndexedPoint = std::pair<RTreePoint, size_t>;
using RTreeIndexedPoints = std::vector<RTreeIndexedPoint>;
using RTreeBox = bg::model::box<RTreePoint>;
using RTreeRing = bg::model::ring<RTreePoint>;
using RTree = bg::index::rtree<RTreeIndexedPoint, bg::index::rstar<16>>;

using NodeId = size_t;
using Node = geom::Pose;
using Nodes = std::vector<Node>;

using VertexId = size_t;

struct Vertex {
    VertexId id;
    NodeId node_id;
    double yaw;

    double start_cost;
    double finish_cost;
    std::optional<VertexId> prev_vertex_id;
};

using VerticesMap = std::unordered_map<NodeId, std::vector<VertexId>>; 

struct NeighborInfo {
    NodeId node_id;
    double yaw;
    double edge_len;
};

using NeighborsInfo = std::vector<NeighborInfo>;

struct GraphParams {
    struct CircularSector {
        double angle;
        double radius;
    } circular_sector;

    struct Spline {
        double step;
        double gamma_ratio;
    } spline;

    struct Yaw {
        size_t front_count;
        size_t back_count;
        double max_shift_abs;
    } yaw;

    double obstacle_dist;
};

class Graph {
  public:
    Graph(const GraphParams& params);

    Graph& setNodes(const geom::Poses& nodes);
    Graph& setCollisionChecker(std::shared_ptr<const collision::StaticCollisionChecker> checker);

    const Nodes& getNodes() const;
    const Node& getNode(NodeId node_id) const;

    double findNearestNodeYaw(NodeId node_id, double yaw) const;

    std::vector<NodeId> findNodesKNN(const geom::Vec2& point, size_t k) const;
    std::vector<NodeId> findNodesInsideCircle(const geom::Circle& circle) const;

    NeighborsInfo findNeighbors(NodeId origin_node_id, double origin_yaw) const;

    geom::Poses findEdge(NodeId from_node, double from_yaw, NodeId to_node, double to_yaw) const;

    double distance(NodeId from_node, NodeId to_node) const;

  private:
    std::vector<double> findNodeYaws(NodeId node_id) const;

    double findEdgeLen(const geom::Poses& edge) const;

    bool isEdgeCollisionFree(const geom::Poses& edge) const;

    Nodes nodes_;
    RTree rtree_;

    std::shared_ptr<const collision::StaticCollisionChecker> checker_ = nullptr;

    GraphParams params_;
};

struct SearcherParams {
    int max_vertices_count;
    double finish_radius;
};

class Searcher {
  public:
    Searcher(const SearcherParams& params);

    Searcher& setStart(const geom::Pose& start);
    Searcher& setFinish(const geom::Vec2& finish);
    Searcher& setGraph(std::shared_ptr<const Graph> graph);

    std::optional<geom::Polyline> findTrajectory();

    NodeId getStartNodeId() const;
    std::unordered_set<NodeId> getFinishNodesIds() const;

  private:
    void initializeNodeInfo();

    void addVertex(const Vertex& vertex);
    VertexId getOptimalVertexId();
    std::optional<VertexId> tryGetVertexId(NodeId node_id, double yaw, const double eps = 1e-4);

    bool isVertexFinish(const Vertex& vertex) const;
    bool isVertexInOpenSet(const Vertex& vertex) const;

    void updateVertexState(Vertex& vertex, double start_cost, VertexId prev_vertex_id);

    geom::Polyline extractTrajectory(VertexId vertex_id);

    geom::Pose start_;
    geom::Vec2 finish_;

    struct NodeInfo {
        NodeId start_id;
        NodeId finish_id;
        std::unordered_set<NodeId> finish_ids;
    } node_info_;

    std::vector<Vertex> vertices_;
    VerticesMap vertices_map_;
    std::set<std::pair<double, VertexId>> open_set_;

    std::shared_ptr<const Graph> graph_ = nullptr;

    SearcherParams params_;
};

}  // namespace truck::routing_planner::search