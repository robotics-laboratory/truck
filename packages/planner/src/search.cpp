#include "planner/search.h"

namespace truck::planner::search {

GridBuilder::GridBuilder(const GridParams& params) {
    width_ = params.width;
    height_ = params.height;
    resolution_ = params.resolution;
}

GridBuilder& GridBuilder::setEgoPose(const geom::Vec2& ego_pose) {
    ego_pose_ = ego_pose;
    return *this;
}

GridBuilder& GridBuilder::setFinishArea(const geom::Circle& finish_area) {
    finish_area_ = finish_area;
    return *this;
}

bool GridBuilder::insideFinishArea(const geom::Vec2& point) {
    return (pow(finish_area_.center.x - point.x, 2) + pow(finish_area_.center.y - point.y, 2)) < pow(finish_area_.radius, 2);
}

std::vector<Node> GridBuilder::build() {
    std::vector<Node> nodes;

    geom::Vec2 node_point_origin(
        -(width_ / 2) + ego_pose_.x,
        -(height_ / 2) + ego_pose_.y
    );

    for (int i = 0; i < int(height_ / resolution_); i++) {
        for (int j = 0; j < int(width_ / resolution_); j++) {
            Node node;
            node.id = NodeId{j, i};
            node.point = node_point_origin + (geom::Vec2(j, i) * resolution_);
            node.is_finish = insideFinishArea(node.point);

            /** @todo node.is_obstacle */

            nodes.push_back(node);
        }
    }

    return nodes;
}

Grid::Grid(const std::vector<Node>& nodes) {
    nodes_ = nodes;
}

YawBins::YawBins(const std::string& path) {
    /** @todo */
}

DynamicGraph::DynamicGraph(std::shared_ptr<Grid> grid) {
    grid_ = grid;
}

Searcher::Searcher(std::shared_ptr<DynamicGraph> graph) {
    graph_ = graph;
}

std::vector<geom::Vec2> Searcher::getPath() {
    /** @todo */
}

}