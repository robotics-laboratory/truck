#include "visualization_node.h"
#include "color.h"

#include "common/math.h"
#include "geom/angle.h"
#include "geom/arc.h"
#include "geom/msg.h"
#include "geom/pose.h"
#include "geom/segment.h"

#include <boost/assert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>

#include <functional>
#include <utility>
#include <cmath>

namespace truck::visualization {

using namespace std::placeholders;

using namespace geom::literals;

VisualizationNode::VisualizationNode() : Node("visualization") {
    model_ = model::makeUniquePtr(
        this->get_logger(),
        Node::declare_parameter<std::string>("model_config", "model.yaml"));

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    params_ = Parameters{
        .ttl = rclcpp::Duration::from_seconds(this->declare_parameter("ttl", 1.0)),

        .ego_z_lev = this->declare_parameter("ego.z_lev", 0.0),
        .ego_height = this->declare_parameter("ego.height", 0.2),

        .ego_track_width = this->declare_parameter("ego.track.width", 0.06),
        .ego_track_height = this->declare_parameter("ego.track.height", 0.01),
        .ego_track_ttl =
            rclcpp::Duration::from_seconds(this->declare_parameter("ego.track.ttl", 2.00)),

        .arc_z_lev = this->declare_parameter("arc.z_lev", 0.0),
        .arc_width = this->declare_parameter("arc.width", 0.06),
        .arc_length = this->declare_parameter("arc.length", 1.0),

        .waypoints_z_lev = this->declare_parameter("waypoints.z_lev", 0.50),
        .waypoints_radius = this->declare_parameter("waypoints.radius", 0.10),

        .trajectory_z_lev = this->declare_parameter("trajectory.z_lev", 0.0),
        .trajectory_width = this->declare_parameter("trajector.width", 0.12),
    };

    cache_.ego_body_scale = geometry_msgs::msg::Vector3();
    cache_.ego_body_scale.x = model_->shape().length;
    cache_.ego_body_scale.y = model_->shape().width;
    cache_.ego_body_scale.z = params_.ego_height;
    cache_.ego_wheel_scale = geometry_msgs::msg::Vector3();
    cache_.ego_wheel_scale.x = 2 * model_->wheel().radius;
    cache_.ego_wheel_scale.y = 2 * model_->wheel().radius;
    cache_.ego_wheel_scale.z = model_->wheel().width;
    cache_.ego_wheel_position_z = params_.ego_z_lev 
        + model_->wheel().radius - params_.ego_height / 2;
    cache_.ego_front_x_offset 
        = model_->wheelBase().length - model_->wheelBase().base_to_rear;
    cache_.ego_rear_x_offset = -model_->wheelBase().base_to_rear;
    cache_.ego_y_offset = model_->wheelBase().width / 2;

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

    slot_.telemetry = Node::create_subscription<truck_msgs::msg::HardwareTelemetry>(
        "/hardware/telemetry",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleTelemetry, this, _1));

    slot_.odom = Node::create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleOdometry, this, _1));

    slot_.trajectory = Node::create_subscription<truck_msgs::msg::Trajectory>(
        "/motion/trajectory",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleTrajectory, this, _1));

    signal_.ego = Node::create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/ego", rclcpp::QoS(1).reliability(qos));

    signal_.ego_track = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/ego/track", rclcpp::QoS(1).reliability(qos));

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
    ++state_.odom_seq_id;

    publishEgo();
    publishEgoTrack();
    publishArc();
}

namespace {

std_msgs::msg::ColorRGBA modeToColor(
    const truck_msgs::msg::ControlMode::ConstSharedPtr& mode) {
    return color::make(*mode);
}

} // namespace

std_msgs::msg::ColorRGBA VisualizationNode::velocityToColor(double velocity, double alpha) const {
    const auto [v_min, v_max] = model_->baseVelocityLimits();
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
    state_.trajectory = std::move(msg);
    publishTrajectory();
}

void VisualizationNode::publishTrajectory() const {
    if (!state_.trajectory) {
        return;
    }

    visualization_msgs::msg::Marker msg;
    msg.header = state_.trajectory->header;
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.frame_locked = true;
    msg.lifetime = params_.ttl;

    msg.scale.x = params_.trajectory_width;
    msg.pose.position.z = params_.trajectory_z_lev;

    msg.points.reserve(state_.trajectory->states.size());
    msg.colors.reserve(state_.trajectory->states.size());

    bool collision = false;
    for (const auto& state: state_.trajectory->states) {
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

namespace {

void addEgoPart(
    visualization_msgs::msg::MarkerArray &msg_array,
    int32_t marker_type,
    const geometry_msgs::msg::Vector3 &scale,
    const std_msgs::msg::ColorRGBA &color) {

    visualization_msgs::msg::Marker msg;
    msg.type = marker_type;
    // always keep last ego marker

    msg.scale = scale;
    msg.color = color;

    msg_array.markers.push_back(msg);
}

void transformMarker(visualization_msgs::msg::Marker &marker, 
    double x, double y, double x_angle, double z_angle) {

    tf2::Quaternion rotation = tf2::Quaternion::getIdentity();
    rotation.setRPY(x_angle, 0, z_angle);
    const tf2::Transform ego_to_base {
        rotation,
        tf2::Vector3{x, y, 0}
    };
    
    tf2::toMsg(ego_to_base, marker.pose);
}

void addEgoWheels(
    visualization_msgs::msg::MarkerArray &msg_array,
    const geometry_msgs::msg::Vector3 &scale,
    const std_msgs::msg::ColorRGBA &color,
    double front_x_offset, double rear_x_offset, double y_offset,
    double right_steering, double left_steering) {

    const int first_id = msg_array.markers.size();
    for (int i = 0; i < 4; ++i) {
        addEgoPart(msg_array,
            visualization_msgs::msg::Marker::CYLINDER, 
            scale, color);
    }

    // Front right wheel.
    transformMarker(msg_array.markers[first_id], 
        front_x_offset, y_offset, M_PI_2, right_steering);

    // Front left wheel.
    transformMarker(msg_array.markers[first_id], 
        front_x_offset, -y_offset, M_PI_2, left_steering);

    // Rear right wheel.
    transformMarker(msg_array.markers[first_id], 
        rear_x_offset, y_offset, M_PI_2, 0);

    // Rear left wheel.
    transformMarker(msg_array.markers[first_id], 
        rear_x_offset, -y_offset, M_PI_2, 0);
}

} // namespace

void VisualizationNode::publishEgo() const {
    if (!state_.odom || !state_.mode) {
        return;
    }

    visualization_msgs::msg::MarkerArray msg_array;
    addEgoPart(msg_array, visualization_msgs::msg::Marker::CUBE, 
        cache_.ego_body_scale, modeToColor(state_.mode));
    addEgoWheels(msg_array, cache_.ego_wheel_scale, 
        modeToColor(state_.mode), cache_.ego_front_x_offset,
        cache_.ego_rear_x_offset, cache_.ego_y_offset,
        state_.telemetry->current_right_steering,
        state_.telemetry->current_left_steering);

    tf2::Quaternion rotation = tf2::Quaternion::getIdentity();
    rotation.setRPY(0, 0, 
        truck::geom::toAngle(state_.odom->pose.pose.orientation).radians());
    tf2::Transform base_to_odom;
    tf2::fromMsg(state_.odom->pose.pose, base_to_odom);

    const int array_size = msg_array.markers.size();
    for (int i = 0; i < array_size; ++i) {
        msg_array.markers[i].id = i;
        msg_array.markers[i].header = state_.odom->header;
        msg_array.markers[i].action 
            = visualization_msgs::msg::Marker::ADD;
        msg_array.markers[i].frame_locked = true;
        tf2::Transform ego_to_base;
        tf2::fromMsg(msg_array.markers[i].pose, ego_to_base);
        tf2::toMsg(base_to_odom * ego_to_base * tf2::Transform(rotation), 
            msg_array.markers[i].pose);
    }

    signal_.ego->publish(msg_array);
}

void VisualizationNode::publishEgoTrack() const {
    if (!state_.odom || !state_.mode || state_.odom_seq_id % params_.ego_track_rate != 0) {
        return;
    }

    visualization_msgs::msg::Marker msg;
    msg.id = state_.odom_seq_id;
    msg.header = state_.odom->header;
    msg.type = visualization_msgs::msg::Marker::SPHERE;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.frame_locked = true;
    msg.lifetime = params_.ego_track_ttl;

    msg.scale.x = params_.ego_track_width;
    msg.scale.y = params_.ego_track_width;
    msg.scale.z = params_.ego_track_height;
    msg.pose = state_.odom->pose.pose;
    msg.pose.position.z = params_.ego_z_lev;
    msg.color = modeToColor(state_.mode);

    signal_.ego_track->publish(msg);
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
    msg.frame_locked = true;
    msg.lifetime = params_.ttl;

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
        RCLCPP_WARN(
            get_logger(),
            "Expected 'base' frame for control, but got %s. Ignore message!",
            state_.control->header.frame_id.c_str());
        return;
    }

    // publish control only for odometry update
    state_.control = std::move(control);
}

void VisualizationNode::publishWaypoints() const {
    if (!state_.waypoints) {
        return;
    }

    const double size = 2 * params_.waypoints_radius;

    visualization_msgs::msg::Marker marker;

    marker.header = state_.waypoints->header;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.frame_locked = true;
    marker.lifetime = params_.ttl;

    marker.points.resize(state_.waypoints->waypoints.size());
    for (size_t i = 0; i < state_.waypoints->waypoints.size(); ++i) {
        const auto& waypoint = state_.waypoints->waypoints[i];

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
    state_.waypoints = std::move(msg);
    publishWaypoints();
}

void VisualizationNode::handleTelemetry(truck_msgs::msg::HardwareTelemetry::ConstSharedPtr msg) {
    state_.telemetry = msg;
}

}  // namespace truck::visualization
