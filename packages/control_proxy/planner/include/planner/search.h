#pragma once

#include "geom/pose.h"
#include "geom/circle.h"

#include <map>
#include <memory>
#include <vector>
#include <string>

namespace truck::planner::search {

using Poses = std::vector<geom::Pose>;

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

class GridBuilder {
    public:
        GridBuilder(const GridParams& params);
        
        GridBuilder& setEgoPose(const geom::Vec2& ego_pose);
        GridBuilder& setFinishArea(const geom::Circle& finish_area);

        std::vector<Node> build();
    private:
        int width_;
        int height_;
        float resolution_;
        geom::Vec2 ego_pose_;
        geom::Circle finish_area_;
        /** @todo static_collision_checker */

        bool insideFinishArea(const geom::Vec2& point);
};

class Grid {
    public:
        Grid(const std::vector<Node>& nodes);
    private:
        std::vector<Node> nodes_;
};

class YawBins {
    public:
        YawBins(const std::string& path);
    private:
        std::vector<geom::Vec2> yaws_;
};

struct EdgeGeometryId {
    size_t offset_x;
    size_t offset_y;
    size_t from_yaw;
    size_t to_yaw;
};

class EdgeGeometryCache {
    public:
        /** @todo */
    private:
        std::map<EdgeGeometryId, Poses> cache_;
};

class DynamicGraph {
    public:
        DynamicGraph(std::shared_ptr<Grid> grid);
    private:
        std::shared_ptr<Grid> grid_; 
};

class Searcher {
    public:
        Searcher(std::shared_ptr<DynamicGraph> graph);

        std::vector<geom::Vec2> getPath();
    private:
        std::shared_ptr<DynamicGraph> graph_;
};

}