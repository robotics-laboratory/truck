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
    geom::Vec2 grid_origin_point =  \
        snapPoint(ego_pose_) - geom::Vec2(params_.width, params_.height) * (params_.resolution / 2);

    for (size_t i = 0; i < params_.height; i++) {
        for (size_t j = 0; j < params_.width; j++) {
            // initialize node
            Node node = Node{
                .index = (i * params_.width) + j,
                .point = grid_origin_point + (geom::Vec2(j, i) * params_.resolution),
                .collision = (checker_->Distance(node.point) < (shape_.width / 2)) ? true : false};

            node_cache_.nodes.emplace_back(node);
            node_cache_.indexed_point_rtree.insert(
                std::make_pair(RTreePoint(node.point.x, node.point.y), node.index));
        }
    }

    calculateNodeTypes();
    return *this;
}

const geom::Pose& Grid::getEgoPose() const { return ego_pose_; }

const std::vector<Node>& Grid::getNodes() const { return node_cache_.nodes; }

const Node& Grid::getNodeByIndex(size_t index) const { return node_cache_.nodes[index]; }

const std::optional<size_t>& Grid::getEgoNodeIndex() const { return node_cache_.index.ego; }

const std::optional<size_t>& Grid::getFinishNodeIndex() const { return node_cache_.index.finish; }

const std::unordered_set<size_t>& Grid::getFinishAreaNodesIndices() const {
    return node_cache_.index.finish_area;
}

geom::Vec2 Grid::snapPoint(const geom::Vec2& point) const {
    return geom::Vec2(
        round<double>(point.x / params_.resolution) * params_.resolution,
        round<double>(point.y / params_.resolution) * params_.resolution);
}

void Grid::calculateNodeTypes() {
    calculateEgoNode();
    calculateFinishNode();
    calculateFinishAreaNodes();
}

void Grid::calculateEgoNode() {
    std::vector<RTreeIndexedPoint> rtree_indexed_points;

    node_cache_.indexed_point_rtree.query(
        bg::index::nearest(RTreePoint(ego_pose_.pos.x, ego_pose_.pos.y), 1), std::back_inserter(rtree_indexed_points));

    int node_index = rtree_indexed_points.back().second;
    node_cache_.nodes[node_index].ego = true;
    node_cache_.index.ego = node_index;
}

void Grid::calculateFinishNode() {
    std::vector<RTreeIndexedPoint> rtree_indexed_points;

    node_cache_.indexed_point_rtree.query(
        bg::index::nearest(RTreePoint(finish_area_.center.x, finish_area_.center.y), 1), std::back_inserter(rtree_indexed_points));

    int node_index = rtree_indexed_points.back().second;
    node_cache_.nodes[node_index].finish = true;
    node_cache_.index.finish = rtree_indexed_points.back().second;
}

void Grid::calculateFinishAreaNodes() {
    std::vector<RTreeIndexedPoint> rtree_indexed_points;

    geom::Vec2 finish_node_point = getNodeByIndex(node_cache_.index.finish.value()).point;

    RTreeBox rtree_finish_box(
        RTreePoint(
            finish_node_point.x - finish_area_.radius,
            finish_node_point.y - finish_area_.radius),
        RTreePoint(
            finish_node_point.x + finish_area_.radius,
            finish_node_point.y + finish_area_.radius));

    node_cache_.indexed_point_rtree.query(
        bg::index::intersects(rtree_finish_box) &&
        bg::index::satisfies([&](RTreeIndexedPoint const& rtree_indexed_point) {
            geom::Vec2 node_point(
                rtree_indexed_point.first.get<0>(),
                rtree_indexed_point.first.get<1>()
            );

            return (finish_node_point - node_point).lenSq() < squared(finish_area_.radius);
        }),
        std::back_inserter(rtree_indexed_points));

    for (const auto& rtree_indexed_point : rtree_indexed_points) {
        int node_index = rtree_indexed_point.second;
        node_cache_.nodes[node_index].finish_area = true;
        node_cache_.index.finish_area.insert(node_index);
    }
}

}  // namespace truck::planner::search
