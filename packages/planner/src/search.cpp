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

size_t Grid::getNodeIndexByPoint(const geom::Vec2& point) const {
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

    for (size_t i = 0; i < params_.height; i++) {
        for (size_t j = 0; j < params_.width; j++) {
            // initialize node
            Node node = Node{
                .index = (i * params_.width) + j,
                .point = origin_clipped + (geom::Vec2(j, i) * params_.resolution),
                .collision = (checker_->distance(node.point) < (shape_.width / 2)) ? true : false};

            cache_.nodes.emplace_back(node);
            cache_.rtree.insert(std::make_pair(Point(node.point.x, node.point.y), node.index));
        }
    }

    /*
    if (!finishPointInsideBorders(finish_area_.center)) {
        return false;
    }
    */

    calculateNodesIndices();
    return true;
}


double Vertex::SearchState::getTotalCost() const {
    return start_cost + heuristic_cost;
}


size_t EdgeCache::getYawsCount() const { return edge_params_.yaws_count; }

size_t EdgeCache::getYawIndexFromAngle(double theta) const {
    // convert angle rad from interval [-PI; PI) to [0; 2PI)
    double theta_shifted = theta + M_PI;

    VERIFY(theta_shifted >= 0 && theta_shifted < 2 * M_PI);

    double yaw_step = full_angle / edge_params_.yaws_count;

    // get index of the yaw angle closest to a given 'theta' angle
    size_t index = round<size_t>(theta_shifted / yaw_step);

    if (index == 12) { index = 0; }

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

EdgesInfo PrimitiveCache::getEdgesInfoByYawIndex(size_t yaw_index) const {
    EdgesInfo edges_info;

    for (size_t edge_index : edges_indices_[yaw_index]) {
        const Edge& edge = edges_[edge_index];

        edges_info.push_back(
            EdgeInfo{
                .index = edge_index,
                .finish_point = edge.poses.back().pos,
                .finish_yaw_index = edge.finish_yaw_index,
                .len = edge.len
            }
        );
    }

    return edges_info;
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
                    geom::AngleVec2(geom::Angle(theta[i]))
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


Searcher::Searcher() {}

void Searcher::setParams(const SearcherParams& params) {
    params_ = params;
}

void Searcher::setGrid(std::shared_ptr<const Grid> grid) {
    grid_ = std::move(grid);
}

void Searcher::setEdgeCache(std::shared_ptr<const EdgeCache> edge_cache) {
    edge_cache_ = std::move(edge_cache);
}

void Searcher::setCollisionChecker(std::shared_ptr<const collision::StaticCollisionChecker> checker) {
    checker_ = std::move(checker);
}

double Searcher::calculateHeuristic(size_t node_index) const {
    const geom::Vec2& from = grid_->getNodeByIndex(node_index).point;
    const geom::Vec2& to = grid_->getNodeByIndex(grid_->getFinishNodeIndex()).point;
    return geom::len(from - to);
}

geom::Poses Searcher::getPath() const { return current_state_.constructPath(grid_->getEgoPose()); }

void Searcher::addVertex(const Vertex& vertex) {
    size_t index = vertices_.size();
    vertices_.push_back(vertex);
    open_set_.insert(index);
    vertices_indices_[vertex.yaw_index].push_back(index);
}

size_t Searcher::getVertexIndexFromOpenSet() const {
    size_t min_cost_index = *open_set_.begin();
    double min_cost_value = vertices_[min_cost_index].state.getTotalCost();

    for (size_t index : open_set_) {
        double current_cost_value = vertices_[index].state.getTotalCost();

        if (current_cost_value < min_cost_value) {
            min_cost_value = current_cost_value;
            min_cost_index = index;
        }
    }

    return min_cost_index;
}

bool Searcher::collisionFree(const EdgeInfo& edge) const {
    const geom::Vec2& origin = \
        grid_->getNodeByIndex(vertices_[current_vertex_index_].node_index).point;

    for (const geom::Pose& pose : edge_cache_->getPosesByEdgeIndex(edge.index)) {
        if (checker_->distance(geom::Pose(pose.pos + origin, pose.dir)) < params_.min_obstacle_distance) {
            return false;
        }
    }

    return true;
}

bool Searcher::insideBorders(const EdgeInfo& edge) const {
    const geom::Vec2& origin = \
        grid_->getNodeByIndex(vertices_[current_vertex_index_].node_index).point;

    geom::Vec2 edge_global_finish_pos = origin + edge.finish_point;
    geom::Vec2 nearest_node_pos = grid_->getNodeByIndex(grid_->getNodeIndexByPoint(edge_global_finish_pos)).point;

    return geom::len(edge_global_finish_pos - nearest_node_pos) < params_.max_node_position_error;
}

std::optional<size_t> Searcher::getVertexIndex(size_t yaw_index, size_t node_index) const {
    std::optional<size_t> index = std::nullopt;

    for (size_t current_index : vertices_indices_[yaw_index]) {
        if (vertices_[current_index].node_index == node_index) {
            index = current_index;
        }
    }

    return index;
}


bool Searcher::State::empty() const {
    return edges.size() == 0;
}

void Searcher::State::clear() {
    edges.clear();
    rtree_poses.clear();
}

void Searcher::State::reverse() {
    std::reverse(edges.begin(), edges.end());

    for (size_t i = 0; i < edges.size(); i++) {
        for (size_t j = 0; j < edges[i].size(); j++) {
            rtree_poses.insert(std::make_pair(Point(edges[i][j].pos.x, edges[i][j].pos.y), i));
        }
    }
}

void Searcher::State::addEdge(const geom::Poses& edge, const geom::Vec2& origin) {
    geom::Poses new_edge;
    size_t edge_index = edges.size();

    for(const geom::Pose& pose : edge) {
        geom::Pose global_pose(
            pose.pos + origin,
            pose.dir
        );

        new_edge.push_back(global_pose);
        // rtree_poses.insert(std::make_pair(Point(global_pose.pos.x, global_pose.pos.y), edge_index));
    }

    edges.push_back(new_edge);
}

void Searcher::State::addEdgeFromPrevState(const geom::Poses& edge) {
    std::vector<geom::Poses> edges_updated;
    edges_updated.push_back(edge);
    edges_updated.insert(edges_updated.end(), edges.begin(), edges.end());
    edges = edges_updated;

    rtree_poses.clear();

    for (size_t i = 0; i < edges.size(); i++) {
        for (const geom::Pose& pose : edges[i]) {
            rtree_poses.insert(std::make_pair(Point(pose.pos.x, pose.pos.y), i));
        }
    }
}

size_t Searcher::State::getNearestEdgeIndexByEgo(const geom::Pose& ego) const {
    std::vector<IndexedPoint> indexed_points;
    
    rtree_poses.query(
        bg::index::nearest(
            Point(ego.pos.x, ego.pos.y), 1
        ),
        std::back_inserter(indexed_points)
    );

    return indexed_points.back().second;
}

geom::Pose Searcher::State::getStartPoseByEgo(const geom::Pose& ego) const {
    return edges[getNearestEdgeIndexByEgo(ego)].back();
}

geom::Poses Searcher::State::constructPath(const geom::Pose& ego) const {
    geom::Poses path;
    RTree rtree_path;

    for (size_t i = 0; i < edges.size(); i++) {
        bool next_edge_exist = (i + 1) < edges.size();
        size_t max_j = edges[i].size();
        if (next_edge_exist) { max_j -= 1; }

        for (size_t j = 0; j < max_j; j++) {
            rtree_path.insert(std::make_pair(Point(edges[i][j].pos.x, edges[i][j].pos.y), path.size()));
            path.push_back(edges[i][j]);
        }
    }

    if (edges.size() > 0) {
        std::vector<IndexedPoint> indexed_points;

        rtree_path.query(
            bg::index::nearest(Point(ego.pos.x, ego.pos.y), 1), std::back_inserter(indexed_points)
        );

        size_t index = indexed_points.back().second;
        path.erase(path.begin(), path.begin() + index);
    }

    return path;
}

void Searcher::buildState() {
    current_state_.clear();

    Vertex& current_vertex = vertices_[current_vertex_index_];
    bool prev_vertex_exist = current_vertex.state.prev_vertex_index.has_value();

    while (prev_vertex_exist) {
        const Vertex& prev_vertex = vertices_[current_vertex.state.prev_vertex_index.value()];
        geom::Vec2 prev_vertex_pos = grid_->getNodeByIndex(prev_vertex.node_index).point;

        geom::Poses edge_poses = edge_cache_->getPosesByEdgeIndex(current_vertex.state.prev_edge_index.value());

        current_state_.addEdge(edge_poses, prev_vertex_pos);

        current_vertex = prev_vertex;
        prev_vertex_exist = current_vertex.state.prev_vertex_index.has_value();
    }

    current_state_.reverse();

    if (!prev_state_.empty()) {
        size_t edge_index = prev_state_.getNearestEdgeIndexByEgo(grid_->getEgoPose());
        current_state_.addEdgeFromPrevState(prev_state_.edges[edge_index]);
    }

    prev_state_ = current_state_;
}

void Searcher::reset() {
    prev_state_.clear();
}

void Searcher::resetVertices() {
    open_set_.clear();
    closed_set_.clear();    

    vertices_.clear();
    vertices_indices_ = std::vector<std::vector<size_t>>(edge_cache_->getYawsCount());
}

void Searcher::Stopwatch::start() {
    t1 = std::chrono::high_resolution_clock::now();
}

void Searcher::Stopwatch::end() {
    t2 = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

    RCLCPP_INFO_STREAM(
        rclcpp::get_logger(""),
        "Path search time: " + std::to_string(time) + "ms";
    );
}

bool Searcher::findPath() {
    stopwatch_.start();
    resetVertices();

    if (!prev_state_.empty()) {
        start_pose = prev_state_.getStartPoseByEgo(grid_->getEgoPose());
    } else {
        start_pose = grid_->getEgoPose();
    }

    double start_theta = start_pose.dir.angle().radians();
    size_t start_yaw_index = edge_cache_->getYawIndexFromAngle(start_theta);
    size_t start_node_index = grid_->getNodeIndexByPoint(start_pose.pos);

    Vertex start_vertex {
        .yaw_index = start_yaw_index,
        .node_index = start_node_index,

        .state = Vertex::SearchState {
            .start_cost = 0.0,
            .heuristic_cost = calculateHeuristic(start_node_index),
            .prev_vertex_index = std::nullopt,
            .prev_edge_index = std::nullopt
        }
    };

    addVertex(start_vertex);

    while(open_set_.size() > 0 && vertices_.size() < params_.max_vertices_count) {
        current_vertex_index_ = getVertexIndexFromOpenSet();

        size_t yaw_index = vertices_[current_vertex_index_].yaw_index;
        size_t node_index = vertices_[current_vertex_index_].node_index;

        const auto& finish_nodes_indices = grid_->getFinishAreaNodesIndices();

        if (finish_nodes_indices.find(node_index) != finish_nodes_indices.end()) {
            buildState();
            stopwatch_.end();
            return true;
        }

        closed_set_.insert(current_vertex_index_);
        open_set_.erase(current_vertex_index_);

        for (const EdgeInfo& edge : edge_cache_->getEdgesInfoByYawIndex(yaw_index)) {
            if (collisionFree(edge) && insideBorders(edge)) {
                size_t neighbor_vertex_yaw_index = edge.finish_yaw_index;

                size_t neighbor_vertex_node_index = grid_->getNodeIndexByPoint(
                    edge.finish_point + grid_->getNodeByIndex(node_index).point
                );

                std::optional<size_t> neighbor_vertex_index = \
                    getVertexIndex(neighbor_vertex_yaw_index, neighbor_vertex_node_index);

                if (!neighbor_vertex_index.has_value()) {
                    Vertex neighbor_vertex {
                        .yaw_index = neighbor_vertex_yaw_index,
                        .node_index = neighbor_vertex_node_index,

                        .state = Vertex::SearchState {
                            .start_cost = vertices_[current_vertex_index_].state.start_cost + edge.len,
                            .heuristic_cost = calculateHeuristic(neighbor_vertex_node_index),
                            .prev_vertex_index = current_vertex_index_,
                            .prev_edge_index = edge.index
                        }
                    };

                    addVertex(neighbor_vertex);
                } else {
                    Vertex& neighbor_vertex = vertices_[neighbor_vertex_index.value()];

                    double new_start_cost = \
                        vertices_[current_vertex_index_].state.start_cost + edge.len;

                    if (new_start_cost < neighbor_vertex.state.start_cost) {
                        neighbor_vertex.state = Vertex::SearchState {
                            .start_cost = new_start_cost,
                            .heuristic_cost = calculateHeuristic(neighbor_vertex.node_index),
                            .prev_vertex_index = current_vertex_index_,
                            .prev_edge_index = edge.index
                        };
                    }
                }
            }
        }
    }

    current_state_ = prev_state_;

    stopwatch_.end();
    return false;
}

}  // namespace truck::planner::search
