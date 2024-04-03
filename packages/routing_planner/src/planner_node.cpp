#include "routing_planner/planner_node.h"

#include "geom/msg.h"
#include "geom/distance.h"

#include <boost/geometry.hpp>

namespace truck::routing_planner::node {

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace bg = boost::geometry;

using RTreePoint = bg::model::point<double, 2, bg::cs::cartesian>;
using RTreeIndexedPoint = std::pair<RTreePoint, size_t>;
using RTreeIndexedPoints = std::vector<RTreeIndexedPoint>;
using RTreeBox = bg::model::box<RTreePoint>;
using RTree = bg::index::rtree<RTreeIndexedPoint, bg::index::rstar<16>>;

namespace {

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

RTreeIndexedPoints RTreeSearchInsideBox(
    const RTree& rtree, const geom::Vec2& center, double radius) {
    RTreeIndexedPoints rtree_indexed_points;

    RTreeBox rtree_box(
        RTreePoint(center.x - radius, center.y - radius),
        RTreePoint(center.x + radius, center.y + radius));

    rtree.query(bg::index::intersects(rtree_box), std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points;
}

geom::Polyline toPolyline(const truck_msgs::msg::NavigationRoute& msg) {
    geom::Polyline polyline;

    for (const auto& point : msg.data) {
        polyline.emplace_back(geom::toVec2(point));
    }

    return polyline;
}

geom::Poses buildRoutingMesh(
    const geom::Polyline& route, const RoutingMeshParams& params, const double threshold = 0.05) {
    geom::Poses routing_mesh;
    size_t half_points_count = floor<size_t>(0.5 * params.width / params.step);

    auto addOrthogonalPoses = [&](const geom::Pose& origin) {
        routing_mesh.emplace_back(origin);

        for (size_t i = 1; i < half_points_count + 1; i++) {
            geom::AngleVec2 PI_2 = geom::AngleVec2(geom::PI_2);

            geom::Pose orthogonal_pose_left = {
                .pos = origin.pos + (origin.dir - PI_2) * (i * params.step), .dir = origin.dir};

            geom::Pose orthogonal_pose_right = {
                .pos = origin.pos + (origin.dir + PI_2) * (i * params.step), .dir = origin.dir};

            routing_mesh.emplace_back(orthogonal_pose_left);
            routing_mesh.emplace_back(orthogonal_pose_right);
        }
    };

    auto route_uniform_it = route.ubegin(params.offset);
    geom::Pose cur_route_pose = *route_uniform_it;

    addOrthogonalPoses(cur_route_pose);

    route_uniform_it++;

    geom::Pose next_route_pose = *route_uniform_it;

    while (params.offset - geom::distance(cur_route_pose.pos, next_route_pose.pos) < threshold) {
        addOrthogonalPoses(next_route_pose);
        cur_route_pose = next_route_pose;
        route_uniform_it++;
        next_route_pose = *route_uniform_it;
    }

    return routing_mesh;
}

geom::Poses maskRoutingMesh(
    const geom::Poses& routing_mesh, const geom::Vec2& mask_center, double mask_radius) {
    RTree rtree = toRTree(routing_mesh);

    geom::Poses routing_mesh_masked;

    for (const auto& rtree_indexes_point : RTreeSearchInsideBox(rtree, mask_center, mask_radius)) {
        routing_mesh_masked.emplace_back(routing_mesh[rtree_indexes_point.second]);
    }

    return routing_mesh_masked;
}

}  // namespace

PlannerNode::PlannerNode() : Node("routing_planner") {
    initializeParams();
    initializeTopicHandlers();

    model_ = std::make_unique<model::Model>(model::load(this->get_logger(), params_.model_config));
    checker_ = std::make_shared<collision::StaticCollisionChecker>(model_->shape());

    timer_ = this->create_wall_timer(250ms, std::bind(&PlannerNode::makePlannerTick, this));
}

void PlannerNode::initializeParams() {
    params_.model_config = this->declare_parameter<std::string>("model_config");

    params_.radius = this->declare_parameter<double>("radius");

    params_.graph = {
        .circular_sector =
            {.angle = this->declare_parameter<double>("graph.circular_sector.angle"),
             .radius = this->declare_parameter<double>("graph.circular_sector.radius")},

        .spline =
            {.step = this->declare_parameter<double>("graph.spline.step"),
             .gamma_ratio = this->declare_parameter<double>("graph.spline.gamma_ratio")},

        .yaw =
            {.front_count =
                 static_cast<size_t>(this->declare_parameter<int>("graph.yaw.front_count")),
             .back_count =
                 static_cast<size_t>(this->declare_parameter<int>("graph.yaw.back_count")),
             .max_shift_abs = this->declare_parameter<double>("graph.yaw.max_shift_abs")},

        .obstacle_dist = this->declare_parameter<double>("graph.obstacle_dist")};

    params_.searcher = {
        .max_vertices_count =
            static_cast<int>(this->declare_parameter<int>("searcher.max_vertices_count")),
        .finish_radius = this->declare_parameter<double>("searcher.finish_radius")};

    params_.routing_mesh = {
        .step = this->declare_parameter<double>("routing_mesh.step"),
        .width = this->declare_parameter<double>("routing_mesh.width"),
        .offset = this->declare_parameter<double>("routing_mesh.offset")};
}

void PlannerNode::initializeTopicHandlers() {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slots_.occupancy_grid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/grid", 1, std::bind(&PlannerNode::onOccupancyGrid, this, _1));

    slots_.odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&PlannerNode::onOdometry, this, _1));

    slots_.finish = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&PlannerNode::onFinish, this, _1));

    slots_.route = Node::create_subscription<truck_msgs::msg::NavigationRoute>(
        "/navigation/route",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&PlannerNode::onRoute, this, _1));

    signals_.mesh = Node::create_publisher<truck_msgs::msg::PlannerMesh>(
        "/planner/mesh", rclcpp::QoS(1).reliability(qos));

    signals_.trajectory = Node::create_publisher<truck_msgs::msg::PlannerTrajectory>(
        "/planner/trajectory", rclcpp::QoS(1).reliability(qos));
}

void PlannerNode::onOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    checker_->reset(collision::distanceTransform(collision::Map::fromOccupancyGrid(*msg)));
}

void PlannerNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    state_.ego = geom::toPose(*msg);
}

void PlannerNode::onFinish(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    state_.finish = geom::toVec2(*msg);
}

void PlannerNode::onRoute(const truck_msgs::msg::NavigationRoute::SharedPtr msg) {
    state_.route = toPolyline(*msg);
}

void PlannerNode::makePlannerTick() {
    if (!state_.ego.has_value() || !state_.finish.has_value() ||
        !state_.route.has_value() || !checker_->initialized()) {
        return;
    }

    geom::Poses routing_mesh_masked = maskRoutingMesh(
        buildRoutingMesh(*state_.route, params_.routing_mesh), (*state_.ego).pos, params_.radius);

    graph_ = std::make_shared<search::Graph>(
        search::Graph(params_.graph)
            .setCollisionChecker(checker_)
            .setNodes(routing_mesh_masked));

    searcher_ = std::make_unique<search::Searcher>(
        search::Searcher(params_.searcher)
            .setStart(*state_.ego)
            .setFinish(*state_.finish)
            .setGraph(graph_));

    state_.trajectory = searcher_->findTrajectory();

    if (!state_.trajectory.has_value()) {
        RCLCPP_INFO(this->get_logger(), "Trajectory not found.");
    }

    publishMesh();
    publishTrajectory();
}

void PlannerNode::publishMesh() {
    truck_msgs::msg::PlannerMesh msg;

    const auto& nodes = graph_->getNodes();
    const auto& start_id = searcher_->getStartNodeId();
    const auto& finish_ids = searcher_->getFinishNodesIds();

    for (size_t i = 0; i < nodes.size(); i++) {
        msg.data.emplace_back(geom::msg::toPoint(nodes[i]));

        if (checker_->distance(nodes[i].pos) < params_.graph.obstacle_dist) {
            msg.obstacles_ids.emplace_back(i);
        }
    }

    msg.start_id = start_id;

    for (const auto finish_id : finish_ids) {
        msg.finish_ids.emplace_back(finish_id);
    }

    signals_.mesh->publish(msg);
}

void PlannerNode::publishTrajectory() {
    truck_msgs::msg::PlannerTrajectory msg;

    msg.status = state_.trajectory.has_value();

    if (state_.trajectory.has_value()) {
        for (const geom::Vec2& point : *state_.trajectory) {
            msg.data.emplace_back(geom::msg::toPoint(point));
        }
    }

    signals_.trajectory->publish(msg);
}

}  // namespace truck::routing_planner::node