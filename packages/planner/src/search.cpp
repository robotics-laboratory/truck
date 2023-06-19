#include "planner/search.h"

using json = nlohmann::json;

namespace truck::planner::search {

Grid::Grid(const GridParams& params) : params(params) {}

Grid& Grid::setEgoPose(const std::optional<const geom::Pose>& ego_pose) {
    ego_pose_ = ego_pose;
    return *this;
}

Grid& Grid::setFinishArea(const std::optional<const geom::Circle>& finish_area) {
    finish_area_ = finish_area;
    return *this;
}

Grid& Grid::setCollisionChecker(std::shared_ptr<const collision::StaticCollisionChecker> checker) {
    this->checker = checker;
    return *this;
}

Grid& Grid::build() {
    // determining clipped ego point
    geom::Vec2 ego_point_clipped = clipPoint(ego_pose_.value());

    // determining clipped origin point
    geom::Vec2 origin_point_clipped = clipPoint(
        ego_pose_.value().pos -
        geom::Vec2(params.width, params.height) * (params.resolution / 2));

    // determining clipped finish point
    geom::Vec2 finish_point_clipped = clipPoint(finish_area_.value().center);

    NodeId ego_id = NodeId{
        .x = int((ego_point_clipped.x - origin_point_clipped.x) / params.resolution),
        .y = int((ego_point_clipped.y - origin_point_clipped.y) / params.resolution)};

    NodeId finish_id = NodeId{
        .x = int((finish_point_clipped.x - origin_point_clipped.x) / params.resolution),
        .y = int((finish_point_clipped.y - origin_point_clipped.y) / params.resolution)};

    for (int i = 0; i < params.height; i++) {
        for (int j = 0; j < params.width; j++) {
            // initialize node
            Node node = Node{
                .id = NodeId{j, i},
                .point = origin_point_clipped + (geom::Vec2(j, i) * params.resolution),
                .is_finish = insideFinishArea(node.point),
                .collision = (checker->distance(node.point) < 1e-5) ? true : false};

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

const std::optional<geom::Pose>& Grid::getEgoPose() const { return ego_pose_; }

const std::vector<Node>& Grid::getNodes() const { return nodes_; }

const std::optional<size_t>& Grid::getStartNodeIndex() const { return start_node_index_; }

const std::optional<size_t>& Grid::getEndNodeIndex() const { return end_node_index_; }

const std::set<size_t>& Grid::getFinishAreaNodesIndices() const { return finish_area_nodes_indices_; }

const Node& Grid::getNodeById(const NodeId& id) const {
    for (const Node& cur_node : nodes_) {
        if (cur_node.id == id) {
            return cur_node;
        }
    }

    // catch case if node wasn't found
    // we guarantee that this should not happen
    assert(false);
}

bool Grid::insideFinishArea(const geom::Vec2& point) const {
    return (finish_area_.value().center - point).lenSq() < squared(finish_area_.value().radius);
}

geom::Vec2 Grid::clipPoint(const geom::Vec2& point) const {
    return geom::Vec2(
        round<double>(point.x / params.resolution) * params.resolution,
        round<double>(point.y / params.resolution) * params.resolution);
}


EdgeGeometryCache::EdgeGeometryCache() {}

EdgeGeometryCache::EdgeGeometryCache(const std::string& path) {
    std::ifstream f(path);
    json data = json::parse(f);

    for (double yaw : data["yaws"]) {
        yaws_.push_back(geom::Angle(yaw));
    }

    assert(yaws_.size() == 12);

    json primitives = data["primitives"];

    for (size_t i = 0; i < primitives.size(); i++) {
        std::string primitive_key = "primitive_" + std::to_string(i);

        Primitive primitive{
            .shift_node_id =
                {.x = primitives[primitive_key]["neighbor_id"]["x"],
                 .y = primitives[primitive_key]["neighbor_id"]["y"]},
            .length = primitives[primitive_key]["length"]};

        json poses = primitives[primitive_key]["poses"];

        for (size_t j = 0; j < poses.size(); j++) {
            std::string pose_key = "pose_" + std::to_string(j);

            geom::Vec2 pos(poses[pose_key]["x"], poses[pose_key]["y"]);
            geom::Angle angle(poses[pose_key]["yaw"]);
            geom::Vec2 dir(angle);

            primitive.poses.push_back(geom::Pose(pos, dir));
        }

        primitives_.push_back(primitive);
    }

    assert(primitives_.size() == 36);
}

size_t EdgeGeometryCache::getIndexByYaw(const geom::Angle& yaw) const {
    size_t index = 0;
    double min_diff = abs((yaws_[index] - yaw).radians());

    for (size_t i = 1; i < yaws_.size(); i++) {
        double cur_diff = abs((yaws_[i] - yaw).radians());

        if (cur_diff < min_diff) {
            min_diff = cur_diff;
            index = i;
        }
    }

    return index;
}

const std::vector<Primitive>& EdgeGeometryCache::getPrimitives() const { return primitives_; }


double VertexSearchState::getTotalCost() const { return start_cost + heuristic_cost; }


void Vertex::updateState(const VertexSearchState& state) { this->state = state; }


DynamicGraph::DynamicGraph(const GraphParams& params) { this->params = params; }

DynamicGraph& DynamicGraph::setGrid(std::shared_ptr<const Grid> grid) {
    this->grid = grid;
    return *this;
}

DynamicGraph& DynamicGraph::setEdgeGeometryCache(
    std::shared_ptr<const EdgeGeometryCache> edge_geometry_cache) {
    this->edge_geometry_cache = edge_geometry_cache;
    return *this;
}

Vertex DynamicGraph::buildVertex(const NodeId& node_id, size_t yaw_index, const VertexSearchState& state) {
    Vertex vertex{
        .node_id = node_id,
        .yaw_index = yaw_index,
        .state = state
    };

    const std::vector<Primitive>& primitives = edge_geometry_cache->getPrimitives();

    for (size_t i = 0; i < primitives.size(); i++) {
        // add primitive index to a vertex if it satisfies the constraints
        if (checkConstraints(&vertex, primitives[i])) {
            vertex.edges_indices.push_back(i);
        }
    }

    return vertex;
}

bool DynamicGraph::yawMask(const Primitive& primitive, const Vertex* vertex) const {
    geom::Angle start_pose_yaw = primitive.poses[0].dir.angle();
    size_t start_pose_yaw_index = edge_geometry_cache->getIndexByYaw(start_pose_yaw);

    return start_pose_yaw_index == vertex->yaw_index;
}

bool DynamicGraph::boundaryMask(const Primitive& primitive, const Vertex* vertex) const {
    NodeId node_id_of_last_pose{
        .x = vertex->node_id.x + primitive.shift_node_id.x,
        .y = vertex->node_id.y + primitive.shift_node_id.y
    };

    if ((node_id_of_last_pose.x >= 0) &&
        (node_id_of_last_pose.y >= 0) &&
        (node_id_of_last_pose.x < grid->params.width) &&
        (node_id_of_last_pose.y < grid->params.height)) {
        return true;
    }

    return false;
}

bool DynamicGraph::collisionMask(const Primitive& primitive, const Vertex* vertex) const {
    const auto& node = grid->getNodeById(vertex->node_id);

    for (const geom::Pose& pose : primitive.poses) {
        geom::Pose pose_global(node.point + pose.pos, pose.dir);

        if (grid->checker->distance(pose_global) < grid->params.min_obstacle_distance) {
            return false;
        }
    }

    return true;
}

bool DynamicGraph::checkConstraints(const Vertex* vertex, const Primitive& primitive) const {
    return yawMask(primitive, vertex) && boundaryMask(primitive, vertex) && collisionMask(primitive, vertex);
}


Searcher::Searcher() {}

Searcher& Searcher::setGraph(std::shared_ptr<DynamicGraph> graph) {
    graph_ = graph;
    return *this;
}

const std::vector<geom::Vec2>& Searcher::getPath() const { return path_; }

double Searcher::getHeuristic(const geom::Vec2& from, const geom::Vec2& to) const { return geom::len(from - to); }

size_t Searcher::findOptimalVertexIndex() const {
    size_t min_total_cost_index = *open_set_.begin();
    double min_total_cost_value = graph_->vertices[min_total_cost_index].state.getTotalCost();

    for (size_t cur_index : open_set_) {
        double cur_total_cost_value = graph_->vertices[cur_index].state.getTotalCost();

        if (cur_total_cost_value < min_total_cost_value) {
            min_total_cost_value = cur_total_cost_value;
            min_total_cost_index = cur_index;
        }
    }

    return min_total_cost_index;
}

std::optional<size_t> Searcher::getNeighborVertexIndex(const NodeId& node_id, size_t yaw_index) const {
    std::optional<size_t> index;

    for (size_t i = 0; i < graph_->vertices.size(); i++) {
        const Vertex& cur_vertex = graph_->vertices[i];

        if ((cur_vertex.node_id == node_id) && (cur_vertex.yaw_index == yaw_index)) {
            index = i;
        }
    }

    return index;
}

void Searcher::buildPath(size_t cur_vertex_index) {
    size_t vertex_index = cur_vertex_index;

    while (graph_->vertices[vertex_index].state.prev_vertex_index.has_value()) {
        size_t primitive_index =
            graph_->vertices[vertex_index].state.prev_to_cur_vertex_primitive_index.value();

        const Primitive& primitive = graph_->edge_geometry_cache->getPrimitives()[primitive_index];

        size_t prev_vertex_index = graph_->vertices[vertex_index].state.prev_vertex_index.value();
        geom::Vec2 offset =
            graph_->grid->getNodeById(graph_->vertices[prev_vertex_index].node_id).point;

        /** @details: 
         *  for correct path drawing all path points
         *  should be added to 'path_' array sequentially
         * 
         *  since primitives are restored from the end,
         *  then within one primitive it's necessary to iterate over its points from the end,
         *  otherwise path visualization will be incorrect
        */
        for (auto it = primitive.poses.rbegin(); it != primitive.poses.rend(); it++) {
            path_.push_back((*it).pos + offset);
        }

        vertex_index = graph_->vertices[vertex_index].state.prev_vertex_index.value();
    }
}

Searcher& Searcher::findPath() {
    /** @details
     *  let's introduce convenient names for objects' references to use
     */
    std::shared_ptr<const Grid> grid = graph_->grid;
    std::vector<Vertex>& vertices = graph_->vertices;

    size_t start_node_index = grid->getStartNodeIndex().value();
    size_t end_node_index = grid->getEndNodeIndex().value();

    const std::set<size_t>& finish_area_nodes_indices = grid->getFinishAreaNodesIndices();

    const Node& start_node = grid->getNodes()[start_node_index];
    const Node& end_node = grid->getNodes()[end_node_index];


    /** @details
     *  check, if we need indeed to search a path
     */
    if (finish_area_nodes_indices.find(start_node_index) != finish_area_nodes_indices.end()) {
        return *this;
    }

    if (!grid->getEndNodeIndex().has_value()) {
        return *this;
    }


    /** @details
     *  initialize searcher, create start vertex
     */
    size_t start_yaw_index = graph_->edge_geometry_cache->getIndexByYaw(
        grid->getEgoPose().value().dir.angle());

    VertexSearchState start_state{
        .start_cost = 0.0,
        .heuristic_cost = getHeuristic(start_node.point, end_node.point),
        .prev_vertex_index = std::nullopt,
        .prev_to_cur_vertex_primitive_index = std::nullopt};

    Vertex start_vertex = graph_->buildVertex(start_node.id, start_yaw_index, start_state);

    vertices.push_back(start_vertex);
    open_set_.insert(0);


    /** @details
     *  loop through 'open_set' while it's not empty
     */
    while (open_set_.size() > 0) {
        size_t cur_vertex_index = findOptimalVertexIndex();

        if (grid->getNodeById(vertices[cur_vertex_index].node_id).is_finish) {
            buildPath(cur_vertex_index);
            return *this;
        }

        closed_set_.insert(cur_vertex_index);
        open_set_.erase(cur_vertex_index);
        
         /** @details
         *  loop through current vertex's existing neighbors
         */
        for (size_t primitive_index : vertices[cur_vertex_index].edges_indices) {
            const Primitive& primitive =
                graph_->edge_geometry_cache->getPrimitives()[primitive_index];

            // find out if neighboring vertex exists
            NodeId neighbor_node_id{
                .x = vertices[cur_vertex_index].node_id.x + primitive.shift_node_id.x,
                .y = vertices[cur_vertex_index].node_id.y + primitive.shift_node_id.y};

            size_t neighbor_yaw_index =
                graph_->edge_geometry_cache->getIndexByYaw(primitive.poses.back().dir.angle());

            std::optional<size_t> neighboring_vertex_index =
                getNeighborVertexIndex(neighbor_node_id, neighbor_yaw_index);

            if (!neighboring_vertex_index.has_value()) {
                // neighbor doesn't exist, need to create one

                const Node& new_vertex_node = grid->getNodeById(neighbor_node_id);

                VertexSearchState new_vertex_state{
                    .start_cost =
                        vertices[cur_vertex_index].state.start_cost + primitive.length,
                    .heuristic_cost = getHeuristic(new_vertex_node.point, end_node.point),
                    .prev_vertex_index = cur_vertex_index,
                    .prev_to_cur_vertex_primitive_index = primitive_index};

                Vertex new_vertex = graph_->buildVertex(neighbor_node_id, neighbor_yaw_index, new_vertex_state);

                vertices.push_back(new_vertex);
                open_set_.insert(vertices.size() - 1);
            } else {
                // neighbor exists
                
                Vertex& neighbor_vertex = vertices[neighboring_vertex_index.value()];

                double new_start_cost = vertices[cur_vertex_index].state.start_cost + primitive.length;

                if (new_start_cost < neighbor_vertex.state.start_cost) {
                    // update neighbor's state
                    VertexSearchState new_state{
                        .start_cost = new_start_cost,
                        .heuristic_cost = neighbor_vertex.state.heuristic_cost,
                        .prev_vertex_index = cur_vertex_index,
                        .prev_to_cur_vertex_primitive_index = primitive_index};

                    neighbor_vertex.updateState(new_state);
                }
            }
        }
    }

    return *this;
}

}  // namespace truck::planner::search