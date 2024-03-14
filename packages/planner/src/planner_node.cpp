#include "planner/planner_node.h"

#include "common/math.h"
#include "geom/rtree.h"
#include "geom/msg.h"
#include "geom/distance.h"

namespace truck::planner::node {

namespace {

geom::Polyline toPolyline(const truck_msgs::msg::NavigationRoute& msg) {
    geom::Polyline polyline;

    for (const auto& point : msg.data) {
        polyline.emplace_back(geom::toVec2(point));
    }

    return polyline;
}

std::vector<geom::Vec2> buildRoutingMesh(
    const RoutingMeshParams& params, const geom::Vec2& ego, const geom::Polyline& route) {
    geom::RTree rtree_mesh_points;
    size_t half_points_count = floor<size_t>(0.5 * params.width / params.dist);

    auto addOrthogonalMeshFromPose = [&](const geom::Pose& pose) {
        rtree_mesh_points.insert(geom::toRTreeIndexedPoint(pose.pos, rtree_mesh_points.size()));

        for (size_t i = 1; i < half_points_count + 1; i++) {
            geom::AngleVec2 PI_2 = geom::AngleVec2(geom::Angle(geom::PI_2));
            geom::Vec2 left = pose.pos + (i * params.dist) * (pose.dir - PI_2);
            geom::Vec2 right = pose.pos + (i * params.dist) * (pose.dir + PI_2);

            rtree_mesh_points.insert(geom::toRTreeIndexedPoint(left, rtree_mesh_points.size()));
            rtree_mesh_points.insert(geom::toRTreeIndexedPoint(right, rtree_mesh_points.size()));
        }
    };

    auto route_uniform_it = route.ubegin(params.offset);
    geom::Pose cur_pose = *route_uniform_it;

    addOrthogonalMeshFromPose(cur_pose);

    route_uniform_it++;
    geom::Pose next_pose = *route_uniform_it;

    const double threshold = 0.05;

    while (params.offset - geom::distance(cur_pose.pos, next_pose.pos) < threshold) {
        addOrthogonalMeshFromPose(next_pose);
        cur_pose = next_pose;
        route_uniform_it++;
        next_pose = *route_uniform_it;
    }

    return geom::toVec2Array(
        geom::RTreeSearchInsideBox(rtree_mesh_points, ego, params.border_radius));
}

std::vector<geom::Vec2> buildLatticeMesh(const LatticeMeshParams& params, const geom::Vec2& ego) {
    std::vector<geom::Vec2> mesh_points;

    auto snapPoint = [&](const geom::Vec2& point) {
        return geom::Vec2(
            round<double>(point.x / params.dist) * params.dist,
            round<double>(point.y / params.dist) * params.dist);
    };

    geom::Vec2 lattice_origin = snapPoint(geom::Vec2(ego.x - params.radius, ego.y - params.radius));
    size_t points_count = floor<size_t>(params.radius / params.dist) * 2 + 1;

    for (size_t i = 0; i < points_count; i++) {
        for (size_t j = 0; j < points_count; j++) {
            geom::Vec2 mesh_point = lattice_origin + (geom::Vec2(i, j) * params.dist);
            mesh_points.emplace_back(mesh_point);
        }
    }

    return mesh_points;
}

}  // namespace

using namespace std::placeholders;
using namespace std::chrono_literals;

PlannerNode::PlannerNode() : Node("planner") {
    initializeParams();
    initializeTopicHandlers();
}

void PlannerNode::initializeParams() {
    bool routing_mode = this->declare_parameter<bool>("routing_mode");
    params_.mode = (routing_mode == true) ? Params::PlannerMode::routingMesh
                                          : Params::PlannerMode::latticeMesh;

    params_.routing_mesh = {
        .width = this->declare_parameter<double>("routing_mesh.width"),
        .dist = this->declare_parameter<double>("routing_mesh.dist"),
        .offset = this->declare_parameter<double>("routing_mesh.offset"),
        .border_radius = this->declare_parameter<double>("routing_mesh.border_radius")};

    params_.lattice_mesh = {
        .dist = this->declare_parameter<double>("lattice_mesh.dist"),
        .radius = this->declare_parameter<double>("lattice_mesh.radius")};

    if (routing_mode == true) {
        RCLCPP_INFO(this->get_logger(), "routing_mesh width: %.2f m", params_.routing_mesh.width);
        RCLCPP_INFO(this->get_logger(), "routing_mesh dist: %.2f m", params_.routing_mesh.dist);
        RCLCPP_INFO(this->get_logger(), "routing_mesh offset: %.2f m", params_.routing_mesh.offset);
        RCLCPP_INFO(
            this->get_logger(),
            "routing_mesh border_radius: %.2f m",
            params_.routing_mesh.border_radius);
    } else {
        RCLCPP_INFO(this->get_logger(), "lattice_mesh dist: %.2f m", params_.lattice_mesh.dist);
        RCLCPP_INFO(this->get_logger(), "lattice_mesh radius: %.2f m", params_.lattice_mesh.radius);
    }
}

void PlannerNode::initializeTopicHandlers() {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slots_.odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&PlannerNode::onOdometry, this, _1));

    slots_.route = Node::create_subscription<truck_msgs::msg::NavigationRoute>(
        "/navigation/route",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&PlannerNode::onRoute, this, _1));

    signals_.mesh = this->create_publisher<visualization_msgs::msg::Marker>(
        "/planner/mesh", rclcpp::QoS(1).reliability(qos));

    signals_.trajectory = this->create_publisher<visualization_msgs::msg::Marker>(
        "/planner/trajectory", rclcpp::QoS(1).reliability(qos));

    timer_ = this->create_wall_timer(250ms, std::bind(&PlannerNode::makePlannerTick, this));
}

void PlannerNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    state_.ego = geom::toPose(*msg);
}

void PlannerNode::onRoute(const truck_msgs::msg::NavigationRoute::SharedPtr msg) {
    state_.route = toPolyline(*msg);
}

void PlannerNode::makePlannerTick() {
    if (!state_.route.has_value() || !state_.ego.has_value()) {
        return;
    }

    if (params_.mode == Params::PlannerMode::routingMesh) {
        state_.mesh = buildRoutingMesh(params_.routing_mesh, (*state_.ego).pos, *state_.route);
    } else {
        state_.mesh = buildLatticeMesh(params_.lattice_mesh, (*state_.ego).pos);
    }

    publishMesh();
}

void PlannerNode::publishMesh() const {
    if (!state_.mesh.has_value()) {
        return;
    }

    visualization_msgs::msg::Marker msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom_ekf";
    msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.color.a = 1.0;
    msg.color.r = 1.0;
    msg.scale.x = 0.1;
    msg.scale.y = 0.1;
    msg.scale.z = 0.1;
    msg.pose.position.z = 0.01;

    for (const auto& point : *state_.mesh) {
        msg.points.emplace_back(geom::msg::toPoint(point));
    }

    signals_.mesh->publish(msg);
}

void PlannerNode::publishTrajectory() const {}

}  // namespace truck::planner::node