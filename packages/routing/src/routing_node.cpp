#include "routing/routing_node.h"

#include "geom/msg.h"
#include "geom/distance.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/unsupported/Eigen/Splines"

namespace truck::routing {

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace {

Eigen::Vector2d toEigenVector2d(const geom::Vec2& point) {
    return Eigen::Vector2d(point.x, point.y);
}

Eigen::MatrixXd toEigenMatrixXd(const std::vector<geom::Vec2>& points) {
    size_t points_count = points.size();
    Eigen::MatrixXd eigen_mat(2, points_count);

    for (size_t i = 0; i < points_count; i++) {
        eigen_mat.col(i) = toEigenVector2d(points[i]);
    }

    return eigen_mat;
}

geom::Vec2 toVec2(const Eigen::Vector2d& vector) { return geom::Vec2(vector.x(), vector.y()); }

geom::Vec2 toVec2(const RTreePoint& rtree_point) {
    return geom::Vec2(rtree_point.get<0>(), rtree_point.get<1>());
}

RTreePoint toRTreePoint(const geom::Vec2& point) { return RTreePoint(point.x, point.y); }

RTreeIndexedPoint toRTreeIndexedPoint(const geom::Vec2& point, size_t index) {
    return RTreeIndexedPoint(toRTreePoint(point), index);
}

RTree toRTree(const std::vector<geom::Vec2>& points) {
    RTree rtree;

    for (size_t i = 0; i < points.size(); i++) {
        rtree.insert(toRTreeIndexedPoint(points[i], i));
    }

    return rtree;
}

RTree toRTree(const navigation::graph::Nodes& nodes) {
    RTree rtree;

    for (const auto& node : nodes) {
        rtree.insert(toRTreeIndexedPoint(node.point, node.id));
    }

    return rtree;
}

RTreePoint findNearestRTreePoint(const RTree& rtree, const geom::Vec2& point) {
    RTreeIndexedPoints rtree_indexed_points;

    rtree.query(
        bg::index::nearest(toRTreePoint(point), 1), std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points[0].first;
}

size_t findNearesIndex(const RTree& rtree, const geom::Vec2& point) {
    RTreeIndexedPoints rtree_indexed_points;

    rtree.query(
        bg::index::nearest(toRTreePoint(point), 1), std::back_inserter(rtree_indexed_points));

    return rtree_indexed_points[0].second;
}

}  // namespace

RoutingNode::RoutingNode() : Node("routing") {
    initializeParams();
    initializeTopicHandlers();

    map_ = std::make_unique<map::Map>(map::Map::fromGeoJson(params_.map_config));
    mesh_builder_ = std::make_unique<navigation::mesh::MeshBuilder>(params_.mesh);
    graph_builder_ = std::make_unique<navigation::graph::GraphBuilder>(params_.graph);

    cache_.graph =
        graph_builder_->build(mesh_builder_->build(map_->polygons()).mesh, map_->polygons());
    cache_.rtree_nodes = toRTree(cache_.graph.nodes);
}

void RoutingNode::initializeParams() {
    params_.map_config = this->declare_parameter<std::string>("map_config");

    params_.route = Params::Route{
        .max_ego_dist = this->declare_parameter<double>("route.max_ego_dist"),
        .postfix_len = this->declare_parameter<double>("route.postfix_len"),
        .spline = Params::Route::Spline{
            .degree = static_cast<size_t>(this->declare_parameter<int>("route.spline.degree")),
            .step = this->declare_parameter<double>("route.spline.step")}};

    params_.mesh = navigation::mesh::MeshParams{
        .dist = this->declare_parameter<double>("nav_mesh.dist"),
        .offset = this->declare_parameter<double>("nav_mesh.offset"),
        .filter = {}};

    bool k_nearest_mode = this->declare_parameter<bool>("graph.k_nearest_mode");
    auto mode = (k_nearest_mode == true) ? navigation::graph::GraphParams::Mode::kNearest
                                         : navigation::graph::GraphParams::Mode::searchRadius;

    params_.graph = {
        .mode = mode,
        .k_nearest = static_cast<size_t>(this->declare_parameter<int>("graph.k_nearest")),
        .search_radius = this->declare_parameter<double>("graph.search_radius")};

    RCLCPP_INFO(this->get_logger(), "route max_ego_dist: %.2f m", params_.route.max_ego_dist);
    RCLCPP_INFO(this->get_logger(), "route postfix_len: %.2f m", params_.route.postfix_len);
    RCLCPP_INFO(this->get_logger(), "route spline degree: %li", params_.route.spline.degree);
    RCLCPP_INFO(this->get_logger(), "route spline step: %.2f m", params_.route.spline.step);

    RCLCPP_INFO(this->get_logger(), "nav_mesh dist: %.2f m", params_.mesh.dist);
    RCLCPP_INFO(this->get_logger(), "nav_mesh offset: %.2f m", params_.mesh.offset);

    if (k_nearest_mode == true) {
        RCLCPP_INFO(this->get_logger(), "graph k_nearest: %li", params_.graph.k_nearest);
    } else {
        RCLCPP_INFO(this->get_logger(), "graph search_radius: %.2f m", params_.graph.search_radius);
    }
}

void RoutingNode::initializeTopicHandlers() {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slots_.odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&RoutingNode::onOdometry, this, _1));

    slots_.finish = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&RoutingNode::onFinish, this, _1));

    signals_.nav_mesh = this->create_publisher<truck_msgs::msg::NavigationMesh>(
        "/navigation_mesh", rclcpp::QoS(1).reliability(qos));

    signals_.route = this->create_publisher<visualization_msgs::msg::Marker>(
        "/navigation/route", rclcpp::QoS(1).reliability(qos));

    signals_.route_smooth = this->create_publisher<visualization_msgs::msg::Marker>(
        "/navigation/route_smooth", rclcpp::QoS(1).reliability(qos));

    timers_.main = this->create_wall_timer(200ms, std::bind(&RoutingNode::routingLoop, this));
    timers_.nav_mesh =
        this->create_wall_timer(1s, std::bind(&RoutingNode::publishNavigationMesh, this));
}

void RoutingNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    state_.ego = geom::toVec2(*msg);
}

void RoutingNode::onFinish(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    state_.finish = geom::toVec2(*msg);

    if (!state_.ego.has_value() || !state_.finish.has_value()) {
        return;
    }

    updateRoute();
}

double RoutingNode::polylineLength(const geom::Polyline& polyline) const {
    double length = 0.0;

    const size_t polyline_size = polyline.size();
    VERIFY(polyline_size > 1);

    for (size_t i = 0; i < polyline_size - 1; i++) {
        length += geom::distance(polyline[i], polyline[i + 1]);
    }

    return length;
}

geom::Polyline RoutingNode::polylineSmooth(const geom::Polyline& polyline) const {
    if (polyline.size() < 1 + params_.route.spline.degree) {
        return polyline;
    }

    geom::Polyline polyline_smooth;

    Eigen::MatrixXd eigen_mat = toEigenMatrixXd(polyline);

    Eigen::Spline<double, 2> spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(
        eigen_mat, params_.route.spline.degree);

    size_t spline_points = ceil<size_t>(polylineLength(polyline) / params_.route.spline.step);
    spline_points = (spline_points < 2) ? 2 : spline_points;

    for (size_t i = 0; i < spline_points; i++) {
        double t = static_cast<double>(i) / (spline_points - 1);
        Eigen::Vector2d eigen_vec = spline(t);
        polyline_smooth.push_back(toVec2(eigen_vec));
    }

    return polyline_smooth;
}

void RoutingNode::updateRoute() {
    navigation::graph::NodeId from = findNearesIndex(cache_.rtree_nodes, state_.ego.value());
    navigation::graph::NodeId to = findNearesIndex(cache_.rtree_nodes, state_.finish.value());

    navigation::search::Path path = navigation::search::findShortestPath(cache_.graph, from, to);

    if (path.trace.size() == 0) {
        RCLCPP_INFO(this->get_logger(), "Route not found.");
        return;
    }

    // update route
    state_.route.polyline = navigation::search::toPolyline(cache_.graph, path);
    state_.route.length = path.length;

    // update smooth route
    state_.route_smooth.polyline = polylineSmooth(state_.route.polyline);
    state_.route_smooth.length = polylineLength(state_.route_smooth.polyline);
    state_.route_smooth.rtree = toRTree(state_.route_smooth.polyline);
}

void RoutingNode::routingLoop() {
    if (!state_.ego.has_value() || !state_.finish.has_value()) {
        return;
    }

    const geom::Vec2 ego_clipped =
        toVec2(findNearestRTreePoint(state_.route_smooth.rtree, state_.ego.value()));

    if (geom::distance(state_.ego.value(), ego_clipped) > params_.route.max_ego_dist) {
        updateRoute();
        RCLCPP_INFO(this->get_logger(), "Ego is far from current route. Update start point.");
    }

    publishRoute();
    publishRouteSmooth();
}

void RoutingNode::publishRoute() const {
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = "odom_ekf";
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.scale.x = 0.18;
    msg.color.a = 0.75;
    msg.color.r = 1.0;
    msg.pose.position.z = 0.01;

    for (const geom::Vec2& point : state_.route.polyline) {
        msg.points.push_back(geom::msg::toPoint(point));
    }

    signals_.route->publish(msg);
}

void RoutingNode::publishRouteSmooth() const {
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = "odom_ekf";
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.scale.x = 0.36;
    msg.color.a = 0.75;
    msg.color.g = 1.0;
    msg.pose.position.z = 0.01;

    for (const geom::Vec2& point : state_.route_smooth.polyline) {
        msg.points.push_back(geom::msg::toPoint(point));
    }

    signals_.route_smooth->publish(msg);
}

void RoutingNode::publishNavigationMesh() const {
    truck_msgs::msg::NavigationMesh msg;

    for (const auto& node : cache_.graph.nodes) {
        msg.points.push_back(geom::msg::toPoint(node.point));
    }

    signals_.nav_mesh->publish(msg);
}

}  // namespace truck::routing