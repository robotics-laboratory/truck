#include "simulator_2d/simulator_node.h"

#include <boost/assert.hpp>

#include <chrono>
#include <functional>

namespace truck::simulator {

using namespace std::placeholders;

SimulatorNode::SimulatorNode() : Node("simulator") {
    model_ = model::makeUniquePtr(
        this->get_logger(),
        Node::declare_parameter<std::string>("model_config", "model.yaml"));

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slot_.control = Node::create_subscription<truck_msgs::msg::Control>(
        "/motion/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&SimulatorNode::handleControl, this, _1));

    signal_.odom = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/simulator/odometry",
        rclcpp::QoS(1).reliability(qos));

    signal_.visualization = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/simulator/visualization", 
        rclcpp::QoS(1).reliability(qos));

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(params_.simulation_tick), 
        std::bind(&SimulatorNode::updateState, this));

    test_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(params_.simulation_tick),
        std::bind(&SimulatorNode::timerCallback, this));

    createTruckMarker();
}

void SimulatorNode::createTruckMarker() {
    truck_.header.frame_id = "odom_ekf";
    truck_.id = 0;

    truck_.type = visualization_msgs::msg::Marker::CUBE;
    truck_.action = visualization_msgs::msg::Marker::ADD;
    truck_.lifetime = rclcpp::Duration::from_seconds(0);

    truck_.pose.position.x = 0.0;
    truck_.pose.position.y = 0.0;
    truck_.pose.position.z = 0.0;

    truck_.scale.x = model_->shape().length;
    truck_.scale.y = model_->shape().width;
    truck_.scale.z = params_.ego_height;

    truck_.color.a = 1.0;
    truck_.color.r = 0.0;
    truck_.color.g = 0.0;
    truck_.color.b = 1.0;
}

void SimulatorNode::updateTruckMarker() {
    truck_.header.stamp = now();
    truck_.pose.position.x = state_.pose.pos.x;
    truck_.pose.position.y = state_.pose.pos.y;
    signal_.visualization->publish(truck_);
}

void SimulatorNode::updateState() {
    state_.pose.pos.x = params_.simulation_tick * control_.velocity;
    updateTruckMarker();
}

void SimulatorNode::handleControl(truck_msgs::msg::Control::ConstSharedPtr control) {
    control_.velocity = control->velocity;
    control_.acceleration = control->acceleration;
    control_.curvature = control->curvature;
}

// For testing.
void SimulatorNode::timerCallback() {
    //*
    control_.velocity = 1.0;
    control_.acceleration = 0.0;
    control_.curvature = 0.0;
    //*/
}

} // namespace truck::simulator