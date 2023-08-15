#include "simulator_2d/simulator_node.h"

#include "geom/vector.h"

#include <boost/assert.hpp>

#include <chrono>
#include <functional>

namespace truck::simulator {

using namespace std::placeholders;

SimulatorNode::SimulatorNode() : Node("simulator") {
    params_ = Parameters{
        .ego_height = this->declare_parameter("ego/height", 0.2),
        .ego_red = (float)this->declare_parameter("ego_red", 0.0),
        .ego_green = (float)this->declare_parameter("ego_green", 0.0),
        .ego_blue = (float)this->declare_parameter("ego_blue", 1.0),
        .update_period = std::chrono::milliseconds(this->declare_parameter<long int>("update_period", 250))
    };

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slots_.control = Node::create_subscription<truck_msgs::msg::Control>(
        "/control/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&SimulatorNode::handleControl, this, _1));

    signals_.odom = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/simulator/odometry",
        rclcpp::QoS(1).reliability(qos));

    signals_.visualization = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/simulator/visualization", 
        rclcpp::QoS(1).reliability(qos));

    auto truck_sizes = engine_.getTruckSizes();
    msgs_.truck.create(signals_.visualization, truck_sizes.x, truck_sizes.y,
        params_.ego_height, params_.ego_red, params_.ego_green, params_.ego_blue);
    createOdometryMessage();

    auto model = model::makeUniquePtr(
        this->get_logger(),
        Node::declare_parameter<std::string>("model_config", "model.yaml"));
    engine_.start(model, this->declare_parameter("simulation_tick", 0.01));
    engine_.setControl(0.0, 0.0, 0.0);

    timer_ = this->create_wall_timer(
        params_.update_period,
        std::bind(&SimulatorNode::publishSignals, this));
}

void SimulatorNode::handleControl(const truck_msgs::msg::Control::ConstSharedPtr control) {
    RCLCPP_INFO_STREAM(this->get_logger(), 
        std::to_string(control->velocity) + " " + std::to_string(control->acceleration) 
            + " " + std::to_string(control->curvature));

    engine_.setControl(control->velocity, control->acceleration, control->curvature);
}

void SimulatorNode::createOdometryMessage() {
    msgs_.odometry.header.frame_id = "odom_ekf";
    msgs_.odometry.pose.pose.position.z = 0.0;
    msgs_.odometry.child_frame_id = "base_link";
}

void SimulatorNode::publishOdometryMessage(const geom::Pose pose, const geom::Vec2 linearVelocity, 
    const geom::Vec2 angularVelocity) {

    msgs_.odometry.header.stamp = now();

    // Set the position.
    msgs_.odometry.pose.pose.position.x = pose.pos.x;
    msgs_.odometry.pose.pose.position.y = pose.pos.y;
    msgs_.odometry.pose.pose.orientation.x = pose.dir.x;
    msgs_.odometry.pose.pose.orientation.y = pose.dir.y;

    // Set the velocity.
    msgs_.odometry.twist.twist.linear.x = linearVelocity.x;
    msgs_.odometry.twist.twist.linear.y = linearVelocity.y;
    msgs_.odometry.twist.twist.angular.x = angularVelocity.x;
    msgs_.odometry.twist.twist.angular.y = angularVelocity.y;

    signals_.odom->publish(msgs_.odometry);
}

void SimulatorNode::publishSignals() {
    auto pose = engine_.getPose();
    auto steering = engine_.getSteering();
    auto linearVelocity = engine_.getLinearVelocity();
    auto angularVelocity = engine_.getAngularVelocity();
    msgs_.truck.publish(pose, steering, now());
    publishOdometryMessage(pose, linearVelocity, angularVelocity);
}

} // namespace truck::simulator