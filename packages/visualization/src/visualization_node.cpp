#include "visualization_node.h"

#include <utility>

namespace truck::visualization {

VisualizationNode::VisualizationNode()
    : Node("visualization_node")
    , model_(Node::declare_parameter<std::string>("model_config", "model.yaml")) {
    RCLCPP_INFO(this->get_logger(), "Model acquired,");

    odometry_slot_ = Node::create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1, std::bind(&VisualizationNode::handleOdometry, this, std::placeholders::_1));

    mode_slot_ = Node::create_subscription<truck_interfaces::msg::ControlMode>(
        "/control/mode", 1, std::bind(&VisualizationNode::handleMode, this, std::placeholders::_1));

    ego_signal_ = Node::create_publisher<visualization_msgs::msg::Marker>("/visualization/ego", 1);
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

void VisualizationNode::handleOdometry(const nav_msgs::msg::Odometry& odom) {

    // if (!mode_) {
    //     return;
    // }

    visualization_msgs::msg::Marker msg;
    msg.pose = odom.pose.pose;
    msg.header = odom.header;
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

void VisualizationNode::handleMode(truck_interfaces::msg::ControlMode::ConstSharedPtr msg) {
    mode_ = std::move(msg);
}

}  // namespace truck::control_proxy