#pragma once

#include "geom/pose.h"
#include "geom/circle.h"
#include "collision/collision_checker.h"

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
    bool is_obstacle;
};

struct GridParams {
    int width;
    int height;
    float resolution;
};

class Grid {
    public:
        Grid(std::vector<Node>& nodes);

        std::vector<Node> getNodes();
    private:
        std::vector<Node> nodes_;
};

class GridBuilder {
    public:
        GridBuilder(const GridParams& params);

        GridBuilder& setEgoPose(const geom::Pose& ego_pose);
        GridBuilder& setFinishArea(const geom::Circle& finish_area);
        GridBuilder& setCollisionChecker(std::shared_ptr<collision::StaticCollisionChecker> collision_checker);

        Grid build();
    private:
        GridParams params_;

        std::optional<geom::Pose> ego_pose_ = std::nullopt;
        std::optional<geom::Circle> finish_area_ = std::nullopt;
        std::shared_ptr<collision::StaticCollisionChecker> collision_checker_;

        bool insideFinishArea(const geom::Vec2& point);
};

class YawBins {
    public:
        YawBins(const std::string& path);
    private:
        std::vector<double> yaws_;
};

class EdgeGeometryCache {
    public:
        EdgeGeometryCache(const std::string& path);
    private:
        std::vector<Primitive> primitives_;
};

struct Vertex {
    NodeId node_id;
    size_t incoming_yaw_index;
    std::vector<Primitive*> edges;
    
    /** @todo
     * add costs with understandable names: f,g,h
     */
};

class DynamicGraph {
    public:
        DynamicGraph();

        DynamicGraph& setGrid(std::shared_ptr<Grid> grid);
        DynamicGraph& setYawBins(std::shared_ptr<YawBins> yaw_bins);
        DynamicGraph& setEdgeGeometryCache(std::shared_ptr<EdgeGeometryCache> edge_geometry_cache);
        DynamicGraph& setCollisionChecker(std::shared_ptr<collision::StaticCollisionChecker> collision_checker);
        std::vector<Vertex> vertices_;

        std::shared_ptr<Grid> grid_;
        std::shared_ptr<collision::StaticCollisionChecker> collision_checker_;
        std::shared_ptr<YawBins> yaw_bins_;
        std::shared_ptr<EdgeGeometryCache> edge_geometry_cache_;
};

class Searcher {
    public:
        Searcher(std::shared_ptr<DynamicGraph> graph);

        void findPath();
        std::vector<geom::Pose> getFoundPath();
    private:
        std::vector<geom::Pose> path_;
        std::shared_ptr<DynamicGraph> graph_;
};

} // namespace truck::planner::search