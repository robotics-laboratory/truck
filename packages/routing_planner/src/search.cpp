#include "routing_planner/search.h"

#include "geom/bezier.h"
#include "geom/distance.h"
#include "common/math.h"
#include "common/exception.h"

namespace truck::routing_planner::search {

namespace {

geom::AngleVec2 toAngleVec2(double rad) { return geom::AngleVec2(geom::Angle(rad)); }

geom::Vec2 toVec2(const RTreePoint& rtree_point) {
    return geom::Vec2(rtree_point.get<0>(), rtree_point.get<1>());
}

RTreePoint toRTreePoint(const geom::Vec2& point) { return RTreePoint(point.x, point.y); }

RTreeIndexedPoint toRTreeIndexedPoint(const geom::Vec2& point, size_t index) {
    return RTreeIndexedPoint(toRTreePoint(point), index);
}

RTree toRTree(const geom::Poses& poses) {
    RTree rtree;

    for (size_t i = 0; i < poses.size(); i++) {
        rtree.insert(toRTreeIndexedPoint(poses[i].pos, i));
    }

    return rtree;
}

RTreeIndexedPoints RTreeSearchKNN(const RTree& rtree, const geom::Vec2& point, size_t k) {
    VERIFY(k > 0);
    RTreeIndexedPoints rtree_indexed_points;

    rtree.query(
        bg::index::nearest(toRTreePoint(point), k), std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points;
}

RTreeIndexedPoints RTreeSearchInsideCircle(const RTree& rtree, const geom::Circle& circle) {
    VERIFY(circle.radius > 0);
    RTreeIndexedPoints rtree_indexed_points;

    RTreeBox rtree_box(
        RTreePoint(circle.center.x - circle.radius, circle.center.y - circle.radius),
        RTreePoint(circle.center.x + circle.radius, circle.center.y + circle.radius));

    rtree.query(
        bg::index::intersects(rtree_box) &&
            bg::index::satisfies([&](RTreeIndexedPoint const& rtree_indexed_point) {
                geom::Vec2 neighbor = toVec2(rtree_indexed_point.first);
                return (circle.center - neighbor).lenSq() < squared(circle.radius);
            }),
        std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points;
}

RTreeIndexedPoints RTreeSearchInsideCircularSector(
    const RTree& rtree, const geom::Vec2& origin_point, double origin_yaw,
    double sector_radius, double sector_angle, const double step = 0.05) {
    VERIFY(sector_radius > 0);
    VERIFY(sector_angle > 0);
    VERIFY(sector_angle <= M_PI * 2);
    VERIFY(step > 0);

    auto buildRTreeRing = [&]() {
        RTreeRing ring;

        size_t points_count = floor<size_t>(sector_angle / step);

        double start_dir = origin_yaw - (sector_angle / 2);
        double end_dir = origin_yaw + (sector_angle / 2);

        geom::Vec2 start_point = origin_point + toAngleVec2(start_dir) * sector_radius;
        geom::Vec2 end_point = origin_point + toAngleVec2(end_dir) * sector_radius;

        bg::append(ring, toRTreePoint(origin_point));
        bg::append(ring, toRTreePoint(start_point));

        for (size_t i = 1; i <= points_count; i++) {
            geom::Vec2 sector_point =
                origin_point + toAngleVec2(start_dir + (step * i)) * sector_radius;
            bg::append(ring, toRTreePoint(sector_point));
        }

        bg::append(ring, toRTreePoint(end_point));
        bg::append(ring, toRTreePoint(origin_point));

        return ring;
    };

    RTreeRing ring = buildRTreeRing();
    RTreeIndexedPoints rtree_indexed_points;

    rtree.query(bg::index::intersects(ring), std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points;
}

}  // namespace

Graph::Graph(const GraphParams& params) : params_(params) {}

Graph& Graph::setNodes(const geom::Poses& nodes) {
    nodes_ = nodes;
    rtree_ = toRTree(nodes);
    return *this;
}

Graph& Graph::setCollisionChecker(
    std::shared_ptr<const collision::StaticCollisionChecker> checker) {
    checker_ = std::move(checker);
    return *this;
}

const Nodes& Graph::getNodes() const { return nodes_; }

const Node& Graph::getNode(NodeId node_id) const { return nodes_[node_id]; }

double Graph::findNearestNodeYaw(NodeId node_id, double yaw) const {
    yaw = geom::Angle::_0_2PI(yaw);

    auto node_yaws = findNodeYaws(node_id);

    double nearest_node_yaw = node_yaws[0];
    double min_diff_abs = std::abs(yaw - nearest_node_yaw);

    for (double cur_node_yaw : node_yaws) {
        double cur_diff_abs = std::abs(yaw - cur_node_yaw);
        if (cur_diff_abs < min_diff_abs) {
            min_diff_abs = cur_diff_abs;
            nearest_node_yaw = cur_node_yaw;
        }
    }

    return nearest_node_yaw;
}

std::vector<NodeId> Graph::findNodesKNN(const geom::Vec2& point, size_t k) const {
    std::vector<NodeId> k_nearest_nodes;

    for (const auto& rtree_indexed_point : RTreeSearchKNN(rtree_, point, k)) {
        k_nearest_nodes.emplace_back(rtree_indexed_point.second);
    }

    return k_nearest_nodes;
}

std::vector<NodeId> Graph::findNodesInsideCircle(const geom::Circle& circle) const {
    std::vector<NodeId> nodes_inside_circle;

    for (const auto& rtree_indexed_point : RTreeSearchInsideCircle(rtree_, circle)) {
        nodes_inside_circle.emplace_back(rtree_indexed_point.second);
    }

    return nodes_inside_circle;
}

NeighborsInfo Graph::findNeighbors(NodeId origin_node_id, double origin_yaw) const {
    NeighborsInfo neighbors_info;

    geom::Vec2 origin_point = nodes_[origin_node_id].pos;

    double origin_yaw_clipped = geom::Angle::_0_2PI(origin_yaw);

    double sector_radius = params_.circular_sector.radius;
    double sector_angle = params_.circular_sector.angle;

    for (const auto& rtree_indexed_point : RTreeSearchInsideCircularSector(
             rtree_, origin_point, origin_yaw_clipped, sector_radius, sector_angle)) {
        NodeId neighbor_node_id = rtree_indexed_point.second;

        if (origin_node_id == neighbor_node_id) {
            continue;
        }

        for (double neighbor_node_yaw : findNodeYaws(neighbor_node_id)) {
            if (std::abs(origin_yaw_clipped - neighbor_node_yaw) > params_.yaw.max_shift_abs) {
                continue;
            }

            geom::Poses edge =
                findEdge(origin_node_id, origin_yaw_clipped, neighbor_node_id, neighbor_node_yaw);

            if (!isEdgeCollisionFree(edge)) {
                continue;
            }

            neighbors_info.emplace_back(NeighborInfo{
                .node_id = neighbor_node_id,
                .yaw = neighbor_node_yaw,
                .edge_len = findEdgeLen(edge)});
        }
    }

    return neighbors_info;
}

double Graph::findEdgeLen(const geom::Poses& edge) const {
    const size_t edge_poses_count = edge.size();
    VERIFY(edge_poses_count > 1);
    double edge_len = 0.0;

    for (size_t i = 0; i < edge_poses_count - 1; i++) {
        edge_len += geom::distance(edge[i].pos, edge[i + 1].pos);
    }

    return edge_len;
}

geom::Poses Graph::findEdge(
    NodeId from_node, double from_yaw, NodeId to_node, double to_yaw) const {
    VERIFY(params_.spline.step > 0);
    VERIFY(params_.spline.gamma_ratio > 0);

    geom::Pose from = {.pos = nodes_[from_node].pos, .dir = toAngleVec2(from_yaw)};
    geom::Pose to = {.pos = nodes_[to_node].pos, .dir = toAngleVec2(to_yaw)};

    double gamma = geom::distance(from.pos, to.pos) * params_.spline.gamma_ratio;

    geom::Vec2 from_ref = from.pos + from.dir * gamma;
    geom::Vec2 to_ref = to.pos - to.dir * gamma;

    const geom::Vec2& p0 = from.pos;
    const geom::Vec2& p1 = from_ref;
    const geom::Vec2& p2 = to_ref;
    const geom::Vec2& p3 = to.pos;

    double dist = (p1 - p0).len() + (p2 - p1).len() + (p3 - p2).len();
    size_t n = 1 + ceil<size_t>(dist / params_.spline.step);
    n = (n > 2) ? n : 3;

    return geom::bezier3(p0, p1, p2, p3, n);
}

std::vector<double> Graph::findNodeYaws(NodeId node_id) const {
    VERIFY(params_.yaw.front_count > 0);

    std::vector<double> yaws;

    double front_angle_step = M_PI / (params_.yaw.front_count + 1);
    double back_angle_step = M_PI / (params_.yaw.back_count + 1);

    double dir = nodes_[node_id].dir.angle().radians();

    double front_angle_start = dir - M_PI_2;
    double back_angle_start = dir - (M_PI_2 * 3);

    for (size_t i = 1; i <= params_.yaw.front_count; i++) {
        double yaw = geom::Angle::_0_2PI(front_angle_start + (i * front_angle_step));
        yaws.emplace_back(yaw);
    }

    for (size_t i = 1; i <= params_.yaw.back_count; i++) {
        double yaw = geom::Angle::_0_2PI(back_angle_start + (i * back_angle_step));
        yaws.emplace_back(yaw);
    }

    return yaws;
}

double Graph::distance(NodeId from_node, NodeId to_node) const {
    return geom::distance(nodes_[from_node].pos, nodes_[to_node].pos);
}

bool Graph::isEdgeCollisionFree(const geom::Poses& edge) const {
    for (const geom::Pose& edge_pose : edge) {
        if (checker_->distance(edge_pose) < params_.obstacle_dist) {
            return false;
        }
    }

    return true;
}

Searcher::Searcher(const SearcherParams& params) : params_(params) {}

Searcher& Searcher::setStart(const geom::Pose& start) {
    start_ = start;
    return *this;
}

Searcher& Searcher::setFinish(const geom::Vec2& finish) {
    finish_ = finish;
    return *this;
}

Searcher& Searcher::setGraph(std::shared_ptr<const Graph> graph) {
    graph_ = std::move(graph);
    return *this;
}

void Searcher::initializeNodeInfo() {
    node_info_.start_id = graph_->findNodesKNN(start_.pos, 1)[0];
    node_info_.finish_id = graph_->findNodesKNN(finish_, 1)[0];

    geom::Vec2 local_finish = graph_->getNode(node_info_.finish_id).pos;
    geom::Circle local_finish_circle = {.center = local_finish, .radius = params_.finish_radius};

    for (const auto node_id : graph_->findNodesInsideCircle(local_finish_circle)) {
        node_info_.finish_ids.emplace(node_id);
    }
}

std::optional<geom::Polyline> Searcher::findTrajectory() {
    initializeNodeInfo();

    Vertex start_vertex = {
        .id = 0,
        .node_id = node_info_.start_id,
        .yaw = graph_->findNearestNodeYaw(node_info_.start_id, start_.dir.angle().radians()),

        .start_cost = 0.0,
        .finish_cost = graph_->distance(node_info_.start_id, node_info_.finish_id),
        .prev_vertex_id = std::nullopt};

    addVertex(start_vertex);

    bool max_vertices_flag =
        (params_.max_vertices_count == -1) ? true : (vertices_.size() < params_.max_vertices_count);

    while (open_set_.size() > 0 && max_vertices_flag) {
        VertexId cur_vertex_id = getOptimalVertexId();
        const Vertex& cur_vertex = vertices_[cur_vertex_id];

        if (isVertexFinish(cur_vertex)) {
            return extractTrajectory(cur_vertex_id);
        }

        for (const auto& neighbor_info :
             graph_->findNeighbors(cur_vertex.node_id, cur_vertex.yaw)) {

            std::optional<VertexId> neighbor_vertex_id =
                tryGetVertexId(neighbor_info.node_id, neighbor_info.yaw);

            if (neighbor_vertex_id.has_value()) {
                Vertex& neighbor_vertex = vertices_[*neighbor_vertex_id];

                if (isVertexInOpenSet(neighbor_vertex)) {
                    double new_start_cost = cur_vertex.start_cost + neighbor_info.edge_len;

                    if (new_start_cost < neighbor_vertex.start_cost) {
                        updateVertexState(neighbor_vertex, new_start_cost, cur_vertex_id);
                    }
                }
            } else {
                Vertex new_neighbor_vertex = {
                    .id = vertices_.size(),
                    .node_id = neighbor_info.node_id,
                    .yaw = neighbor_info.yaw,

                    .start_cost = cur_vertex.start_cost + neighbor_info.edge_len,
                    .finish_cost = graph_->distance(neighbor_info.node_id, node_info_.finish_id),
                    .prev_vertex_id = cur_vertex_id};

                addVertex(new_neighbor_vertex);
            }
        }
    }

    return std::nullopt;
}

NodeId Searcher::getStartNodeId() const { return node_info_.start_id; }

std::unordered_set<NodeId> Searcher::getFinishNodesIds() const { return node_info_.finish_ids; }

void Searcher::addVertex(const Vertex& vertex) {
    vertices_.emplace_back(vertex);
    vertices_map_[vertex.node_id].emplace_back(vertex.id);
    open_set_.insert({vertex.start_cost + vertex.finish_cost, vertex.id});
}

VertexId Searcher::getOptimalVertexId() {
    auto it = open_set_.begin();
    VertexId vertex_id = it->second;
    open_set_.erase(it);
    return vertex_id;
}

std::optional<VertexId> Searcher::tryGetVertexId(NodeId node_id, double yaw, const double eps) {
    auto it = vertices_map_.find(node_id);

    if (it != vertices_map_.end()) {
        for (VertexId vertex_id : (*it).second) {
            if (geom::equal(yaw, vertices_[vertex_id].yaw, eps)) {
                return vertex_id;
            }
        }
    }

    return std::nullopt;
}

bool Searcher::isVertexFinish(const Vertex& vertex) const {
    return node_info_.finish_ids.find(vertex.node_id) != node_info_.finish_ids.end();
}

bool Searcher::isVertexInOpenSet(const Vertex& vertex) const {
    return open_set_.find({vertex.start_cost + vertex.finish_cost, vertex.id}) != open_set_.end();
}

void Searcher::updateVertexState(Vertex& vertex, double start_cost, VertexId prev_vertex_id) {
    open_set_.erase({vertex.start_cost + vertex.finish_cost, vertex.id});

    vertex.start_cost = start_cost;
    vertex.prev_vertex_id = prev_vertex_id;

    open_set_.insert({vertex.start_cost + vertex.finish_cost, vertex.id});
}

// Extract trajectory by traversing vertices in reverse order through the index of previous vertex
geom::Polyline Searcher::extractTrajectory(VertexId vertex_id) {
    geom::Polyline trajectory;

    Vertex& cur_vertex = vertices_[vertex_id];

    while (cur_vertex.prev_vertex_id.has_value()) {
        // Edge should end in current vertex
        NodeId to_node = cur_vertex.node_id;
        double to_yaw = cur_vertex.yaw;

        Vertex& prev_vertex = vertices_[*cur_vertex.prev_vertex_id];

        // Edge should start in previous vertex
        NodeId from_node = prev_vertex.node_id;
        double from_yaw = prev_vertex.yaw;

        auto edge = geom::Polyline(graph_->findEdge(from_node, from_yaw, to_node, to_yaw));

        // Edge points should be reversed for correct trajectory visualization
        std::reverse(edge.begin(), edge.end());

        trajectory.insert(trajectory.end(), edge.begin(), edge.end());

        cur_vertex = prev_vertex;
    }

    // Trajectory points should be reversed, because
    // first trajectory point should have a zero index, but not the last index
    std::reverse(trajectory.begin(), trajectory.end());

    return trajectory;
}

}  // namespace truck::routing_planner::search