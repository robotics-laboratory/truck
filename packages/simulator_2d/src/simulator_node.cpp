#include "simulator_2d/simulator_node.h"

#include "geom/msg.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace truck::simulator {

using namespace std::placeholders;

SimulatorNode::SimulatorNode() : Node("simulator"), 
    engine_(Node::declare_parameter<std::string>("model_config", "model.yaml"), 
        declare_parameter("integration_step", 0.001), 
        declare_parameter("calculations_precision", 1e-8)) {

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slots_.control = Node::create_subscription<truck_msgs::msg::Control>(
        "/control/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&SimulatorNode::handleControl, this, _1));

    signals_.time = Node::create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(1).reliability(qos));

    signals_.odometry = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered", rclcpp::QoS(1).reliability(qos));

    signals_.tf_publisher = Node::create_publisher<tf2_msgs::msg::TFMessage>(
        "/ekf/odometry/transform", rclcpp::QoS(1).reliability(qos));

    signals_.telemetry = Node::create_publisher<truck_msgs::msg::HardwareTelemetry>(
        "/hardware/telemetry", rclcpp::QoS(1).reliability(qos));

    signals_.state = Node::create_publisher<truck_msgs::msg::SimulationState>(
        "/simulator/state", rclcpp::QoS(1).reliability(qos));

    params_ = Parameters{
        .update_period = declare_parameter("update_period", 0.01)};

    timer_ = create_wall_timer(
        std::chrono::duration<double>(params_.update_period),
        std::bind(&SimulatorNode::publishSignals, this));
}

void SimulatorNode::handleControl(const truck_msgs::msg::Control::ConstSharedPtr control) {
    if (control->has_acceleration) {
        engine_.setControl(control->velocity, control->acceleration, control->curvature);
    } else {
        engine_.setControl(control->velocity, control->curvature);
    }
}

void SimulatorNode::publishTime(const rclcpp::Time &time) {
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = time;
    signals_.time->publish(clock_msg);
}

void SimulatorNode::publishOdometryMessage(
    const rclcpp::Time &time, const geom::Pose &pose) {

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "odom_ekf";
    odom_msg.child_frame_id = "odom_ekf";
    odom_msg.header.stamp = time;

    // Set the position.
    odom_msg.pose.pose.position.x = pose.pos.x;
    odom_msg.pose.pose.position.y = pose.pos.y;
    odom_msg.pose.pose.orientation = truck::geom::msg::toQuaternion(pose.dir);

    // Set the velocity.
    const auto linearVelocity = engine_.getLinearVelocity();
    odom_msg.twist.twist.linear.x = linearVelocity.x;
    odom_msg.twist.twist.linear.y = linearVelocity.y;
    tf2::Quaternion orientation;
    tf2::convert(odom_msg.pose.pose.orientation, orientation);
    tf2::Matrix3x3 twist_angles(orientation);
    double roll, pitch, yaw;
    twist_angles.getRPY(roll, pitch, yaw);
    odom_msg.twist.twist.angular.x = roll; 
    odom_msg.twist.twist.angular.y = pitch; 
    odom_msg.twist.twist.angular.z = yaw;

    signals_.odometry->publish(odom_msg);
}

void SimulatorNode::publishTransformMessage(const rclcpp::Time &time, const geom::Pose &pose) {
    geometry_msgs::msg::TransformStamped odom_to_base_transform_msg;
    odom_to_base_transform_msg.header.frame_id = "odom_ekf";
    odom_to_base_transform_msg.child_frame_id = "base";
    odom_to_base_transform_msg.header.stamp = time;

    // Set the transformation.
    odom_to_base_transform_msg.transform.translation.x = pose.pos.x;
    odom_to_base_transform_msg.transform.translation.y = pose.pos.y;
    odom_to_base_transform_msg.transform.rotation = truck::geom::msg::toQuaternion(pose.dir);

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(odom_to_base_transform_msg);
    signals_.tf_publisher->publish(tf_msg);
}

void SimulatorNode::publishTelemetryMessage(const rclcpp::Time &time) {
    truck_msgs::msg::HardwareTelemetry telemetry_msg;
    telemetry_msg.header.frame_id = "base";
    telemetry_msg.header.stamp = time;
    const auto current_steering = engine_.getCurrentSteering();
    telemetry_msg.current_left_steering = current_steering.left.radians();
    telemetry_msg.current_right_steering = current_steering.right.radians();
    const auto target_steering = engine_.getTargetSteering();
    telemetry_msg.target_left_steering = target_steering.left.radians();
    telemetry_msg.target_right_steering = target_steering.right.radians();
    signals_.telemetry->publish(telemetry_msg);
}

void SimulatorNode::publishSimulationStateMessage(const rclcpp::Time &time) {

    truck_msgs::msg::SimulationState state_msg;
    state_msg.header.frame_id = "odom_ekf";
    state_msg.header.stamp = time;
    state_msg.speed = engine_.getTwist().velocity;
    state_msg.steering = engine_.getMiddleSteering();
    signals_.state->publish(state_msg);
}

void SimulatorNode::publishSignals() {
    engine_.advance(params_.update_period);
    const auto time = engine_.getTime();
    const auto pose = engine_.getPose();
    publishTime(time);
    publishOdometryMessage(time, pose);
    publishTransformMessage(time, pose);
    publishTelemetryMessage(time);
    publishSimulationStateMessage(time);
}

}  // namespace truck::simulator
