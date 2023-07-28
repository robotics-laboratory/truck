#include "simulator_2d/simulator_node.h"

#include "geom/vector.h"

#include <boost/assert.hpp>

#include <chrono>
#include <functional>

namespace truck::simulator {

using namespace std::placeholders;

SimulatorNode::SimulatorNode() : Node("simulator") {
    auto model = model::makeUniquePtr(
        this->get_logger(),
        Node::declare_parameter<std::string>("model_config", "model.yaml"));
    engine_ = new SimulatorEngine(model);

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slots_.control = Node::create_subscription<truck_msgs::msg::Control>(
        "/motion/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&SimulatorNode::handleControl, this, _1));

    signals_.odom = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/simulator/odometry",
        rclcpp::QoS(1).reliability(qos));

    signals_.visualization = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/simulator/visualization", 
        rclcpp::QoS(1).reliability(qos));

    timer_ = this->create_wall_timer(
        params_.update_period, 
        std::bind(&SimulatorNode::publishSignals, this));

    createTruckMarker();
    createOdometryMessage();
}

SimulatorNode::~SimulatorNode() {
    delete engine_;
}

void SimulatorNode::handleControl(const truck_msgs::msg::Control::ConstSharedPtr control) const {
    engine_->setControl(control->velocity, control->acceleration, control->curvature);
}

void SimulatorNode::createTruckMarker() {
    msgs_.truck.header.frame_id = "odom_ekf";
    msgs_.truck.id = 0;

    msgs_.truck.type = visualization_msgs::msg::Marker::CUBE;
    msgs_.truck.action = visualization_msgs::msg::Marker::ADD;
    msgs_.truck.lifetime = rclcpp::Duration::from_seconds(0);

    msgs_.truck.pose.position.x = 0.0;
    msgs_.truck.pose.position.y = 0.0;
    msgs_.truck.pose.position.z = 0.0;

    auto truck_sizes = engine_->getTruckSizes();
    msgs_.truck.scale.x = truck_sizes.x;
    msgs_.truck.scale.y = truck_sizes.y;
    msgs_.truck.scale.z = params_.ego_height;

    msgs_.truck.color.a = 1.0;
    msgs_.truck.color.r = 0.0;
    msgs_.truck.color.g = 0.0;
    msgs_.truck.color.b = 1.0;
}

void SimulatorNode::publishTruckMarker(const geom::Pose pose, const geom::Angle steering) {
    msgs_.truck.header.stamp = now();
    msgs_.truck.pose.position.x = pose.pos.x + 0 * steering.radians();
    msgs_.truck.pose.position.y = pose.pos.y;
    signals_.visualization->publish(msgs_.truck);
}

void SimulatorNode::createOdometryMessage() {
    msgs_.odometry.header.frame_id = "odom_ekf";
    msgs_.odometry.pose.pose.position.z = 0.0;
    msgs_.odometry.child_frame_id = "base_link";
}

void SimulatorNode::publishOdometryMessage(const geom::Pose pose, const geom::Angle steering) {
    msgs_.odometry.header.stamp = now();

    // Set the position.
    msgs_.odometry.pose.pose.position.x = pose.pos.x + 0 * steering.radians();
    msgs_.odometry.pose.pose.position.y = pose.pos.y;
    //msgs_.odometry.pose.pose.orientation = odom_quat;

    // Set the velocity.
    //msgs_.odometry.twist.twist.linear.x = vx;
    //msgs_.odometry.twist.twist.linear.y = vy;
    //msgs_.odometry.twist.twist.angular.z = vth;

    signals_.odom->publish(msgs_.odometry);
}

void SimulatorNode::publishSignals() {
    //* For testing
    engine_->setControl(1.0, 0.0, 0.0);
    //*/

    auto pose = engine_->getPose();
    auto steering = engine_->getSteering();
    publishTruckMarker(pose, steering);
    publishOdometryMessage(pose, steering);
}

} // namespace truck::simulator