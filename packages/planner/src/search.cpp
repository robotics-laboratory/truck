#include "planner/search.h"

namespace truck::planner::search {

using nlohmann::json;

Grid::Grid(const GridParams& params, const model::Shape& shape) : params_(params), shape_(shape) {}

Grid& Grid::setEgoPose(const geom::Pose& ego_pose) {
    ego_pose_ = ego_pose;
    return *this;
}

Grid& Grid::setFinishArea(const geom::Square& finish_area) {
    finish_area_ = finish_area;
    return *this;
}

Grid& Grid::setCollisionChecker(std::shared_ptr<const collision::StaticCollisionChecker> checker) {
    checker_ = std::move(checker);
    return *this;
}

const geom::Pose& Grid::getEgoPose() const { return ego_pose_; }

const std::vector<Node>& Grid::getNodes() const { return cache_.nodes; }

const Node& Grid::getNodeByIndex(size_t index) const { return cache_.nodes.at(index); }

size_t Grid::getNodeIndexByPoint(const geom::Vec2& point) {
    std::vector<IndexedPoint> indexed_points;

    cache_.rtree.query(
        bg::index::nearest(Point(point.x, point.y), 1), std::back_inserter(indexed_points));

    return indexed_points.back().second;
}

size_t Grid::getEgoNodeIndex() const { return cache_.ego_index; }

size_t Grid::getFinishNodeIndex() const { return cache_.finish_index; }

const std::unordered_set<size_t>& Grid::getFinishAreaNodesIndices() const {
    return cache_.finish_area_indices;
}

geom::Vec2 Grid::snapPoint(const geom::Vec2& point) const {
    return geom::Vec2(
        round<double>(point.x / params_.resolution) * params_.resolution,
        round<double>(point.y / params_.resolution) * params_.resolution);
}

void Grid::calculateNodesIndices() {
    // find indices of ego and finish nodes
    cache_.ego_index = getNodeIndexByPoint(ego_pose_);
    cache_.finish_index = getNodeIndexByPoint(finish_area_.center);

    // find indices of nodes inside finish area
    std::vector<IndexedPoint> indexed_points;

    const geom::Vec2& finish_node_point = cache_.nodes[cache_.finish_index].point;

    geom::Vec2 shift(finish_area_.size / 2, finish_area_.size / 2);

    geom::Vec2 p1 = finish_node_point - shift;
    geom::Vec2 p2 = finish_node_point + shift;

    Box finish_area_box(Point(p1.x, p1.y), Point(p2.x, p2.y));

    cache_.rtree.query(
        bg::index::intersects(finish_area_box),
        std::back_inserter(indexed_points)
    );

    for (const auto& indexed_point : indexed_points) {
        cache_.finish_area_indices.insert(indexed_point.second);
    }
}

bool Grid::finishPointInsideBorders(const geom::Vec2& point) {
    size_t nearest_node_index = getNodeIndexByPoint(point);
    geom::Vec2 nearest_node_point = cache_.nodes[nearest_node_index].point;

    return geom::len(nearest_node_point - point) < params_.resolution;
}

bool Grid::build() {
    geom::Vec2 ego_clipped = snapPoint(ego_pose_);

    geom::Vec2 origin_clipped = snapPoint(
        ego_clipped - geom::Vec2(params_.width, params_.height) * (params_.resolution / 2));

    for (int i = 0; i < params_.height; i++) {
        for (int j = 0; j < params_.width; j++) {
            // initialize node
            Node node = Node{
                .index = (i * params_.width) + j,
                .point = origin_clipped + (geom::Vec2(j, i) * params_.resolution),
                .collision = (checker_->distance(node.point) < (shape_.width / 2)) ? true : false};

            cache_.nodes.emplace_back(node);
            cache_.rtree.insert(std::make_pair(Point(node.point.x, node.point.y), node.index));
        }
    }

    if (!finishPointInsideBorders(finish_area_.center)) {
        cache_.nodes.clear();
        return false;
    }

    calculateNodesIndices();
    return true;
}

size_t EdgeCache::getYawIndexFromAngle(double theta) {
    // convert angle rad from interval [-PI; PI) to [0; 2PI)
    double theta_shifted = theta + M_PI;

    VERIFY(theta_shifted >= 0 && theta_shifted < 2 * M_PI);

    double yaw_step = full_angle / edge_params_.yaws_count;

    // get index of the yaw angle closest to a given 'theta' angle
    size_t index = round<size_t>(theta_shifted / yaw_step);

    VERIFY(index >= 0 && index < edge_params_.yaws_count);

    return index;
}

const geom::Poses& EdgeCache::getPosesByEdgeIndex(size_t index) const {
    return edges_[index].poses;
}

PrimitiveCache::PrimitiveCache(const EdgeParams& edge_params) {
    edge_params_ = edge_params;
    parseJSON();
}

void PrimitiveCache::parseJSON() {
    std::ifstream file(edge_params_.primitive.json_path);
    json data = json::parse(file);

    edges_indices_ = data["primitives_indices"].get<std::vector<std::vector<size_t>>>();

    for (json::iterator it = data["primitives"].begin(); it != data["primitives"].end(); it++) {
        auto len = (*it)["len"].get<double>();
        auto x = (*it)["x"].get<std::vector<double>>();
        auto y = (*it)["y"].get<std::vector<double>>();
        auto theta = (*it)["theta"].get<std::vector<double>>();

        geom::Poses poses;
        for (int i = 0; i < x.size(); i++) {
            poses.push_back(
                geom::Pose(
                    geom::Vec2(x[i], y[i]),
                    geom::Vec2(geom::Angle(theta[i]))
                )
            );
        }

        edges_.push_back(
            Edge{
                .poses = poses,
                .finish_yaw_index = getYawIndexFromAngle(theta.back()),
                .len = len
            }
        );
    }

    RCLCPP_INFO_STREAM(
        rclcpp::get_logger(""),
        "JSON was successfully parsed. Total primitives count: " + std::to_string(edges_.size())
    );
}

}  // namespace truck::planner::search