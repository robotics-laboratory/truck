#include "planner/search.h"

namespace truck::planner::search {

Grid::Grid(const GridParams& params) : params_(params) {}

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
    geom::Vec2 ego_clipped = snapPoint(ego_pose_);
    geom::Vec2 finish_clipped = snapPoint(finish_area_.center);

    geom::Vec2 origin_clipped = snapPoint(
        ego_clipped - geom::Vec2(params_.width, params_.height) * (params_.resolution / 2));

    NodeId ego_id = toNodeId(ego_clipped, origin_clipped);
    NodeId finish_id = toNodeId(finish_clipped, origin_clipped);

    for (int i = 0; i < params_.height; i++) {
        for (int j = 0; j < params_.width; j++) {
            // initialize node
            Node node = Node{
                .id = NodeId{j, i},
                .point = origin_clipped + (geom::Vec2(j, i) * params_.resolution),
                .is_finish = insideFinishArea(node.point),
                .collision = (checker_->distance(node.point) < 1e-5) ? true : false};

            nodes_.push_back(node);

            size_t cur_node_index = nodes_.size() - 1;

            if (node.id == ego_id) {
                // save index of a start node
                start_node_index_ = cur_node_index;
            }

            if (node.id == finish_id) {
                // save index of an end node
                end_node_index_ = cur_node_index;
            }

            if (node.is_finish) {
                finish_area_nodes_indices_.insert(cur_node_index);
            }
        }
    }

    return *this;
}

const geom::Pose& Grid::getEgoPose() const { return ego_pose_; }

const std::vector<Node>& Grid::getNodes() const { return nodes_; }

const Node& Grid::getNodeById(const NodeId& id) const {
    return nodes_.at(id.x + params_.width * id.y);
}

const std::optional<size_t>& Grid::getStartNodeIndex() const { return start_node_index_; }

const std::optional<size_t>& Grid::getEndNodeIndex() const { return end_node_index_; }

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

NodeId Grid::toNodeId(const geom::Vec2& point, const geom::Vec2& origin) const {
    return NodeId{
        .x = int((point.x - origin.x) / params_.resolution),
        .y = int((point.y - origin.y) / params_.resolution)};
}

}  // namespace truck::planner::search