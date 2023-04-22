#include "visualization_node.h"
#include "color.h"

#include "common/math.h"
#include "geom/angle.h"
#include "geom/arc.h"
#include "geom/msg.h"
#include "geom/pose.h"
#include "geom/segment.h"

#include <boost/assert.hpp>

#include <functional>
#include <utility>

namespace truck::visualization {

using namespace std::placeholders;

using namespace geom::literals;

VisualizationNode::VisualizationNode()
    : Node("visualization")
    , model_(model::load(
          this->get_logger(), Node::declare_parameter<std::string>("model_config", "model.yaml"))) {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    params_ = Parameters {
        .ego_z_lev = this->declare_parameter("ego/z_lev", 0.0),
        .ego_height = this->declare_parameter("ego/height", 0.2),

        .arc_z_lev = this->declare_parameter("arc/z_lev", 0.0),
        .arc_width = this->declare_parameter("arc/width", 0.06),
        .arc_length = this->declare_parameter("arc/length", 1.0),
                
        .waypoints_z_lev = this->declare_parameter("waypoints/z_lev", 0.50),
        .waypoints_radius = this->declare_parameter("waypoints/radius", 0.10),
        
        .trajectory_z_lev = this->declare_parameter("trajectory/z_lev", 0.0),
        .trajectory_width = this->declare_parameter("trajectory/width", 0.12),
    };

    slot_.mode = Node::create_subscription<truck_msgs::msg::ControlMode>(
        "/control/mode",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleMode, this, _1));

    slot_.control = Node::create_subscription<truck_msgs::msg::Control>(
        "/control/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleControl, this, _1));

    slot_.waypoints = Node::create_subscription<truck_msgs::msg::Waypoints>(
        "/waypoints",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleWaypoints, this, _1));

    slot_.odom = Node::create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleOdometry, this, _1));

    slot_.trajectory = Node::create_subscription<truck_msgs::msg::Trajectory>(
        "/motion/trajectory",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleTrajectory, this, _1));

    signal_.ego = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/ego",
        rclcpp::QoS(1).reliability(qos));

    signal_.arc = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/arc",
        rclcpp::QoS(1).reliability(qos));

    signal_.waypoints = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/waypoints",
        rclcpp::QoS(1).reliability(qos));

    signal_.trajectory = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/trajectory",
        rclcpp::QoS(1).reliability(qos));
}

void VisualizationNode::handleOdometry(nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    state_.odom = std::move(odom);

    publishEgo();
    publishControl();
}

namespace {

std_msgs::msg::ColorRGBA modeToColor(
    const truck_msgs::msg::ControlMode::ConstSharedPtr& mode) {
    return color::make(*mode);
}

} // namespace

std_msgs::msg::ColorRGBA VisualizationNode::velocityToColor(double velocity, double alpha) const {
    const auto [v_min, v_max] = model_.baseVelocityLimits();
    BOOST_ASSERT(v_min <= 0 && 0 < v_max);

    const double ratio = [&]{
        if (velocity >= 0) {
            return velocity / v_max;
        }

        return abs(v_min) > 0  ? (velocity / v_min) : 0.0;
    }();

    return color::plasma(1 - ratio, alpha);
}

void VisualizationNode::handleTrajectory(truck_msgs::msg::Trajectory::ConstSharedPtr msg) {
    publishTrajectory(*msg);
}

void VisualizationNode::publishTrajectory(const truck_msgs::msg::Trajectory& trajectory) const {
    visualization_msgs::msg::Marker msg;
    msg.header = trajectory.header;
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;

    msg.scale.x = params_.trajectory_width;
    msg.pose.position.z = params_.trajectory_z_lev;

    msg.points.reserve(trajectory.states.size());
    msg.colors.reserve(trajectory.states.size());

    bool collision = false;
    for (const auto& state: trajectory.states) {
        collision |= state.collision;

        const auto color = collision
            ? color::gray(0.5)
            : velocityToColor(state.velocity, 0.5);

        msg.points.push_back(state.pose.position);
        msg.colors.push_back(color);
    }

    signal_.trajectory->publish(msg);
}

void VisualizationNode::handleMode(truck_msgs::msg::ControlMode::ConstSharedPtr msg) {
    state_.mode = std::move(msg);
    publishEgo();
}

void VisualizationNode::publishEgo() const {
    if (not state_.odom) {
        return;
    }

    visualization_msgs::msg::Marker msg;
    msg.header = state_.odom->header;
    msg.type = visualization_msgs::msg::Marker::CUBE;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.scale.x = model_.shape().length;
    msg.scale.y = model_.shape().width;
    msg.scale.z = params_.ego_height;
    msg.pose = state_.odom->pose.pose;
    msg.pose.position.z = params_.ego_z_lev;
    msg.color = modeToColor(state_.mode);

    signal_.ego->publish(msg);
}

namespace {

geom::Poses arcTrace(const geom::Pose& pose, double curvature, double length) {
    constexpr double step = 0.1;
    constexpr double eps = 1e-2;

    const double abs_curvature = std::abs(curvature);
    if (abs_curvature < eps) {
        return geom::Segment(pose.pos, pose.pos + pose.dir * length).trace(step);
    }

    const geom::Vec2 center = pose.pos + pose.dir.left() / curvature;
    const geom::Vec2 begin = (pose.pos - center).unit();
    const auto delta = clamp(geom::Angle(length * curvature), -geom::PI, geom::PI);

    return geom::Arc(center, abs(1/curvature), begin, delta).trace(step);
}

}  // namespace

void VisualizationNode::publishControl() const {
    publishArc();
}

void VisualizationNode::publishArc() const {
    if (!state_.odom || !state_.control) {
        return;
    }

    std_msgs::msg::Header header;
    header.frame_id = state_.odom->header.frame_id;
    header.stamp = state_.control->header.stamp;

    constexpr double eps = 1e-3;
    if (std::abs(state_.control->velocity) < eps) {
        visualization_msgs::msg::Marker msg;
        msg.header = header;
        msg.action = visualization_msgs::msg::Marker::DELETE;
        signal_.arc->publish(msg);
        return;
    }

    const auto trace = arcTrace(
        geom::toPose(*state_.odom),
        state_.control->curvature,
        params_.arc_length);

    visualization_msgs::msg::Marker msg;

    msg.header = header;
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;

    msg.scale.x = params_.arc_width;
    msg.pose.position.z = params_.arc_z_lev;

    msg.points.reserve(trace.size());
    msg.color = color::white(0.6);

    for (const auto& pose: trace) {
        msg.points.push_back(geom::msg::toPoint(pose.pos));
    }

    signal_.arc->publish(msg);
}


void VisualizationNode::handleControl(truck_msgs::msg::Control::ConstSharedPtr control) {
    if (control->header.frame_id != "base") {
        RCLCPP_WARN(get_logger(), "Expected 'base' frame for cotrol, but got %s. Ignore message!", state_.control->header.frame_id.c_str());
        return;
    }

    state_.control = control;
    publishControl();
}

void VisualizationNode::publishWaypoints(const truck_msgs::msg::Waypoints& msg) const {
    const double size = 2 * params_.waypoints_radius;

    visualization_msgs::msg::Marker marker;

    marker.header = msg.header;

    marker.points.resize(msg.waypoints.size());
    for (size_t i = 0; i < msg.waypoints.size(); ++i) {
        const auto& waypoint = msg.waypoints[i];

        marker.header = msg.header;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        marker.color = color::red();

        marker.points[i] = waypoint;
        marker.points[i].z = params_.waypoints_z_lev;
    }

    signal_.waypoints->publish(marker);
}

void VisualizationNode::handleWaypoints(truck_msgs::msg::Waypoints::ConstSharedPtr msg) {
    publishWaypoints(*msg);
}

}  // namespace truck::control_proxy