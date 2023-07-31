#include "planner/search.h"

namespace truck::planner::search {

Grid::Grid(const GridParams& params, const model::Shape& shape) : params_(params), shape_(shape) {}

Grid& Grid::setEgoPose(const geom::Pose& ego_pose) {
    ego_pose_ = ego_pose;
    return *this;
}

Grid& Grid::setFinishArea(const geom::Circle& finish_area) {
    finish_area_ = finish_area;
    return *this;
}

Grid& Grid::setCollisionChecker(std::shared_ptr<const collision::StaticCollisionChecker> checker) {
    checker_ = std::move(checker);
    return *this;
}

Grid& Grid::build() {
    geom::Vec2 origin = snapPoint(
        snapPoint(ego_pose_) - geom::Vec2(params_.width, params_.height) * (params_.resolution / 2));

    for (int i = 0; i < params_.height; i++) {
        for (int j = 0; j < params_.width; j++) {
            // initialize node
            Node node = Node{
                .index = (params_.width * i) + j,
                .point = origin + (geom::Vec2(j, i) * params_.resolution),
                .finish = insideFinishArea(node.point),
                .collision = (checker_->distance(node.point) < (shape_.width / 2)) ? true : false};

            nodes_.emplace_back(node);
            nodes_points_.insert(std::make_pair(Point(node.point.x, node.point.y), node.index));

            if (node.finish) {
                finish_area_nodes_indices_.insert(node.index);
            }
        }
    }

    std::vector<Value> nearest_ego_values;
    std::vector<Value> nearest_finish_values;

    Point ego_point(ego_pose_.pos.x, ego_pose_.pos.y);
    Point finish_point(finish_area_.center.x, finish_area_.center.y);

    nodes_points_.query(bgi::nearest(ego_point, 1), std::back_inserter(nearest_ego_values));
    nodes_points_.query(bgi::nearest(finish_point, 1), std::back_inserter(nearest_finish_values));

    ego_node_index_ = nearest_ego_values.back().second;
    finish_node_index_ = nearest_finish_values.back().second;

    return *this;
}

const geom::Pose& Grid::getEgoPose() const { return ego_pose_; }

const std::vector<Node>& Grid::getNodes() const { return nodes_; }

const Node& Grid::getNodeByIndex(size_t index) const {
    return nodes_.at(index);
}

const std::optional<size_t>& Grid::getEgoNodeIndex() const { return ego_node_index_; }

const std::optional<size_t>& Grid::getFinishNodeIndex() const { return finish_node_index_; }

const std::unordered_set<size_t>& Grid::getFinishAreaNodesIndices() const {
    return finish_area_nodes_indices_;
}

bool Grid::insideFinishArea(const geom::Vec2& point) const {
    return (finish_area_.center - point).lenSq() < squared(finish_area_.radius);
}

geom::Vec2 Grid::snapPoint(const geom::Vec2& point) const {
    return geom::Vec2(
        round<double>(point.x / params_.resolution) * params_.resolution,
        round<double>(point.y / params_.resolution) * params_.resolution);
}

}  // namespace truck::planner::search