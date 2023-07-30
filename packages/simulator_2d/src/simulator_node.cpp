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
    engine_ = new SimulatorEngine(model, this->declare_parameter("simulation_tick", 0.01));

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

    timer_ = this->create_wall_timer(
        params_.update_period,
        std::bind(&SimulatorNode::publishSignals, this));

    auto truck_sizes = engine_->getTruckSizes();
    msgs_.truck = new TruckMarker(signals_.visualization, truck_sizes.x, truck_sizes.y,
        params_.ego_height, params_.ego_red, params_.ego_green, params_.ego_blue);
    createOdometryMessage();
}

SimulatorNode::~SimulatorNode() {
    delete engine_;
    delete msgs_.truck;
}

void SimulatorNode::handleControl(const truck_msgs::msg::Control::ConstSharedPtr control) const {
    RCLCPP_INFO_STREAM(this->get_logger(), 
        std::to_string(control->velocity) + " " + std::to_string(control->acceleration) 
            + " " + std::to_string(control->curvature));

    engine_->setControl(control->velocity, control->acceleration, control->curvature);
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
    auto pose = engine_->getPose();
    auto steering = engine_->getSteering();
    msgs_.truck->publish(pose, steering, now());
    publishOdometryMessage(pose, steering);
}

} // namespace truck::simulator