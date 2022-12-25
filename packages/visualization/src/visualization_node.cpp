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

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    mode_slot_ = Node::create_subscription<truck_interfaces::msg::ControlMode>(
        "/control/mode",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleMode, this, std::placeholders::_1));

    control_slot_ = Node::create_subscription<truck_interfaces::msg::Control>(
        "/control/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleControl, this, std::placeholders::_1));

    ego_box_signal_ = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/ego/box", rclcpp::QoS(1).reliability(qos));

    ego_circles_signal_ = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/ego/circles", rclcpp::QoS(1).reliability(qos));

    arc_signal_ = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/arc", rclcpp::QoS(1).reliability(qos));

    publishEgo();
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

void VisualizationNode::handleMode(truck_interfaces::msg::ControlMode::ConstSharedPtr msg) {
    mode_ = std::move(msg);
    publishEgo();
}

void VisualizationNode::publishEgo() const {
    visualization_msgs::msg::Marker truck_box;
    truck_box.header.stamp = now();
    truck_box.header.frame_id = "base";
    truck_box.id = 0;
    truck_box.type = visualization_msgs::msg::Marker::CUBE;
    truck_box.action = visualization_msgs::msg::Marker::ADD;
    truck_box.lifetime = rclcpp::Duration::from_seconds(0);
    truck_box.scale.x = model_.truckShape().length;;
    truck_box.scale.y = model_.truckShape().width;
    truck_box.scale.z = 0.2;
    truck_box.pose.position.z = 0.1;
    truck_box.color = modeToColor(mode_);

    ego_box_signal_->publish(truck_box);
    
    visualization_msgs::msg::Marker truck_circles;
    truck_circles.header.stamp = now();
    truck_circles.header.frame_id = "base";
    truck_circles.id = 1;
    truck_circles.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    truck_circles.action = visualization_msgs::msg::Marker::ADD;
    truck_circles.lifetime = rclcpp::Duration::from_seconds(0);
    truck_circles.scale.x = model_.truckShape().width;
    truck_circles.scale.y = model_.truckShape().width;
    truck_circles.scale.z = 0.0001;
    truck_circles.color = modeToColor(mode_);
    
    int k = model_.truckShape().circles_count_approx_shape;
    float l = model_.truckShape().length - model_.truckShape().width;

    if (k == 1) {
        geometry_msgs::msg::Point p;
        truck_circles.points.push_back(p);
    } else {
        for (int i = 0; i < k; i++) {
            geometry_msgs::msg::Point p;
            p.x = - (l / 2) + i * (l / (k - 1));
            truck_circles.points.push_back(p);
        }
    } 

    ego_circles_signal_->publish(truck_circles);
}

namespace {

geom::Polyline trace(const geom::Pose& pose, double length, double step) {
    const geom::Vec2& start = pose.pos;
 
    static constexpr double eps = 1e-3;
    BOOST_VERIFY(eps < length);

    const auto step_n = ceil<size_t>(length / step);
    const geom::Vec2 vector = length / step_n * pose.dir.unit();

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
    constexpr double eps = 1e-6;

    if (!control_ || std::abs(control_->velocity) < eps) {
        visualization_msgs::msg::Marker msg;
        msg.header.stamp = now();
        msg.header.frame_id = "base";
        msg.action = visualization_msgs::msg::Marker::DELETE;
        arc_signal_->publish(msg);
        return;
    }

    BOOST_VERIFY(control_->header.frame_id == "base");

    visualization_msgs::msg::Marker msg;
    msg.header = control_->header;
    msg.id = 0;
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.lifetime = rclcpp::Duration::from_seconds(0);
    msg.scale.x = 0.05;
    msg.pose.position.z = 0.1;
    msg.color.r = 255;
    msg.color.a = 1.0;

    const geom::Pose pose {geom::Vec2{0, 0}, geom::Vec2::axisX()};
    const double curvature = control_->curvature;

    for (const auto& point : trace(pose, curvature)) {
        geometry_msgs::msg::Point point_msg;
        point_msg.x = point.x;
        point_msg.y = point.y;

        msg.points.push_back(point_msg);
    }

    arc_signal_->publish(msg);
}

void VisualizationNode::handleControl(truck_interfaces::msg::Control::ConstSharedPtr control) {
    control_ = control;
    publishArc();
}

}  // namespace truck::control_proxy