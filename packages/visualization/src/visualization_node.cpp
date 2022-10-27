#include "visualization_node.h"

#include "geom/polyline.h"
#include "geom/pose.h"
#include "geom/transform.h"

#include <utility>

namespace truck::visualization {

VisualizationNode::VisualizationNode()
    : Node("visualization_node")
    , model_(Node::declare_parameter<std::string>("model_config", "model.yaml")) {
    RCLCPP_INFO(this->get_logger(), "Model acquired...");

    odometry_slot_ = Node::create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1, std::bind(&VisualizationNode::handleOdometry, this, std::placeholders::_1));

    mode_slot_ = Node::create_subscription<truck_interfaces::msg::ControlMode>(
        "/control/mode", 1, std::bind(&VisualizationNode::handleMode, this, std::placeholders::_1));

    control_slot_ = Node::create_subscription<truck_interfaces::msg::Control>(
        "/control/command", 1, std::bind(&VisualizationNode::handleControl, this, std::placeholders::_1));

    ego_signal_ = Node::create_publisher<visualization_msgs::msg::Marker>("/visualization/ego", 1);
    arc_signal_ = Node::create_publisher<visualization_msgs::msg::Marker>("/visualization/arc", 1);
}

namespace {

std_msgs::msg::ColorRGBA modeToColor(
    const truck_interfaces::msg::ControlMode::ConstSharedPtr& mode) {
    std_msgs::msg::ColorRGBA color;

    color.a = 0.5;
    if (!mode || mode->mode == truck_interfaces::msg::ControlMode::OFF) {
        color.r = 255;
    } else if (mode->mode == truck_interfaces::msg::ControlMode::REMOTE) {
        color.b = 255;
    } else if ((mode->mode == truck_interfaces::msg::ControlMode::AUTO)) {
        color.g = 255;
    } else {
        BOOST_ASSERT_MSG(false, "Unknown mode!");
    }

    return color;
}

} // namespace

void VisualizationNode::handleOdometry(nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    odom_ = odom;

    publishEgo();
    publishArc();
}

void VisualizationNode::handleMode(truck_interfaces::msg::ControlMode::ConstSharedPtr msg) {
    mode_ = std::move(msg);
    publishEgo();
}

void VisualizationNode::publishEgo() const {
    if (!odom_) {
        visualization_msgs::msg::Marker msg;
        msg.action = visualization_msgs::msg::Marker::DELETE;
        ego_signal_->publish(msg);
        return;
    }

    visualization_msgs::msg::Marker msg;
    msg.pose = odom_->pose.pose;
    msg.header = odom_->header;
    msg.id = 0;
    msg.type = visualization_msgs::msg::Marker::CUBE;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.lifetime = rclcpp::Duration::from_seconds(0);
    msg.scale.x = model_.wheelBase().length;
    msg.scale.y = model_.wheelBase().width;
    msg.scale.z = 0.12;
    msg.color = modeToColor(mode_);

    ego_signal_->publish(msg);
}

namespace {

geom::Polyline trace(const geom::Pose& pose, double length, double step) {
    const geom::Vec2& start = pose.pos;

    geom::Vec2 vector = length * pose.dir.unit();

    static constexpr double eps = 1e-3;
    BOOST_VERIFY(eps < length);

    const auto step_n = ceil<size_t>(length / step);
    vector /= step_n;

    geom::Polyline points;
    points.reserve(step_n + 1);

    geom::Vec2 point = start;
    points.push_back(point);

    for (size_t i = 1; i <= step_n; ++i) {
        point += vector;
        points.push_back(point);
    }

    return points;
}

geom::Polyline trace(const geom::Pose& pose, double curvature) {
    constexpr double max_length = 1.6;
    constexpr double step = 0.1;
    constexpr double eps = 1e-2;

    const double abs_curvature = std::abs(curvature);

    if (abs_curvature < eps) {
        return trace(pose, max_length, step);
    }

    const geom::Vec2 center = pose.pos + pose.dir.left() / curvature;

    const auto diff = std::clamp(geom::Angle(max_length * curvature), -geom::PI, geom::PI);

    const size_t step_n = ceil<size_t>(diff.radians() / (step * curvature));
    const geom::Angle angle_step = diff / step_n;

    geom::Vec2 radius = pose.pos - center;

    geom::Polyline points;
    points.push_back(pose.pos);

    for (size_t i = 1; i <= step_n; ++i) {
        radius = radius.rotate(angle_step);
        points.push_back(center + radius);
    }

    return points;
}

}  // namespace

void VisualizationNode::publishArc() const {
    if (!odom_ || !control_) {
        visualization_msgs::msg::Marker msg;
        msg.action = visualization_msgs::msg::Marker::DELETE;
        arc_signal_->publish(msg);
        return;
    }

    visualization_msgs::msg::Marker msg;
    msg.header = odom_->header;
    msg.id = 0;
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.lifetime = rclcpp::Duration::from_seconds(0);
    msg.scale.x = 0.05;
    msg.color.r = 255;
    msg.color.a = 1.0;

    const geom::Pose pose = geom::toPose(odom_->pose.pose);
    const double z = odom_->pose.pose.position.z;
    const double curvature = control_->curvature;

    for (const auto& point : trace(pose, curvature)) {
        geometry_msgs::msg::Point point_msg;
        point_msg.x = point.x;
        point_msg.y = point.y;
        point_msg.z = z;

        msg.points.push_back(point_msg);
    }

    arc_signal_->publish(msg);
}

void VisualizationNode::handleControl(truck_interfaces::msg::Control::ConstSharedPtr control) {
    control_ = control;
    publishArc();
}

}  // namespace truck::control_proxy