#pragma once

#include "geom/pose.h"
#include "geom/square.h"
#include "collision/collision_checker.h"

#include <nlohmann/json.hpp>
#include <boost/geometry.hpp>

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <optional>
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
    int width;
    int height;
    double resolution;
    double finish_area_size;
    double min_obstacle_distance;
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

    size_t getNodeIndexByPoint(const geom::Vec2& point);

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
        std::optional<size_t> prev_vertex_index;
        std::optional<size_t> prev_edge_index;
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

class EdgeCache {
  public:
    size_t getYawIndexFromAngle(double theta);

    const geom::Poses& getPosesByEdgeIndex(size_t index) const;

  protected:
    EdgeParams edge_params_;
    std::vector<Edge> edges_;

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

  private:
    void parseJSON();
};

}  // namespace truck::planner::search