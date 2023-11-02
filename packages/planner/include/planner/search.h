#pragma once

#include "geom/pose.h"
#include "geom/circle.h"
#include "collision/collision_checker.h"

#include <boost/geometry.hpp>

#include <optional>
#include <unordered_set>

namespace truck::planner::search {

namespace bg = boost::geometry;

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
    using RTreePoint = bg::model::point<double, 2, bg::cs::cartesian>;
    using RTreeIndexedPoint = std::pair<RTreePoint, size_t>;
    using RTreeBox = bg::model::box<RTreePoint>;
    using RTree = bg::index::rtree<RTreeIndexedPoint, bg::index::rstar<16>>;

    geom::Vec2 snapPoint(const geom::Vec2& point) const;

    void calculateNodeTypes();
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

struct Vertex {
    size_t node_index;
    size_t yaw_index;

    struct State {
        // path cost from start vertex to this vertex
        double start_cost = 0.0;

        std::optional<size_t> prev_vertex_index;
        std::optional<size_t> prev_edge_index;
    } state;
};

struct Edge {
    size_t index;
    double len;

    struct NeighborVertex {
        size_t node_index;
        size_t yaw_index;
    } neighbor_vertex;
};

class EdgeCache {
  public:
    virtual void reset() = 0;
    virtual std::vector<Edge> getEdgesfromVertex(const Vertex& vertex) = 0;

    void setGrid(std::shared_ptr<const Grid> grid);
    void setCollisionChecker(std::shared_ptr<const collision::StaticCollisionChecker> checker);

  protected:
    std::vector<geom::Poses> cache_;

    std::shared_ptr<const Grid> grid_ = nullptr;
    std::shared_ptr<const collision::StaticCollisionChecker> checker_ = nullptr;
};

struct PrimitiveCacheParams {
    std::string json_path;
};

class PrimitiveCache : public EdgeCache {
  public:
    PrimitiveCache(const PrimitiveCacheParams& params);

    void reset() override;
    std::vector<Edge> getEdgesfromVertex(const Vertex& vertex) override;

  private:
    PrimitiveCacheParams params_;
};

struct SplineCacheParams {
    int yaws_count;
    double sector_angle;
    double sector_radius;
};

class SplineCache : public EdgeCache {
  public:
    SplineCache(const SplineCacheParams& params);

    void reset() override;
    std::vector<Edge> getEdgesfromVertex(const Vertex& vertex) override;

  private:
    SplineCacheParams params_;
};

}  // namespace truck::planner::search