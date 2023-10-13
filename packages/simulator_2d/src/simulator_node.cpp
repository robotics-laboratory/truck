#include "simulator_2d/simulator_node.h"

#include "geom/msg.h"

#include <cmath>

namespace truck::simulator {

using namespace std::placeholders;

SimulatorNode::SimulatorNode() : Node("simulator") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slots_.control = Node::create_subscription<truck_msgs::msg::Control>(
        "/control/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&SimulatorNode::handleControl, this, _1));

    signals_.odometry = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered", rclcpp::QoS(1).reliability(qos));

    signals_.tf_publisher = Node::create_publisher<tf2_msgs::msg::TFMessage>(
        "/ekf/odometry/transform", rclcpp::QoS(1).reliability(qos));

    signals_.telemetry = Node::create_publisher<truck_msgs::msg::HardwareTelemetry>(
        "/hardware/telemetry", rclcpp::QoS(1).reliability(qos));

    signals_.state = Node::create_publisher<truck_msgs::msg::SimulationState>(
        "/simulator/state", rclcpp::QoS(1).reliability(qos));

    params_ = Parameters{
        .update_period = this->declare_parameter("update_period", 0.01),
        .precision = this->declare_parameter("calculations_precision", 1e-8)};

    auto model = model::makeUniquePtr(
        this->get_logger(), Node::declare_parameter<std::string>("model_config", "model.yaml"));
    engine_.start(model, this->declare_parameter("integration_step", 0.001), params_.precision);

    timer_ = this->create_wall_timer(
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

void SimulatorNode::publishOdometryMessage(
    const rclcpp::Time &time, const geom::Pose &pose, const geom::Vec2 &linearVelocity,
    const geom::Vec2 &angularVelocity) {

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "odom_ekf";
    odom_msg.child_frame_id = "odom_ekf";
    odom_msg.header.stamp = time;

    // Set the position.
    odom_msg.pose.pose.position.x = pose.pos.x;
    odom_msg.pose.pose.position.y = pose.pos.y;
    const auto quaternion = truck::geom::msg::toQuaternion(pose.dir);
    odom_msg.pose.pose.orientation.x = quaternion.x;
    odom_msg.pose.pose.orientation.y = quaternion.y;
    odom_msg.pose.pose.orientation.z = quaternion.z;
    odom_msg.pose.pose.orientation.w = quaternion.w;

    // Set the velocity.
    odom_msg.twist.twist.linear.x = linearVelocity.x;
    odom_msg.twist.twist.linear.y = linearVelocity.y;
    odom_msg.twist.twist.angular.x = angularVelocity.x;
    odom_msg.twist.twist.angular.y = angularVelocity.y;

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
    const auto quaternion = truck::geom::msg::toQuaternion(pose.dir);
    odom_to_base_transform_msg.transform.rotation.x = quaternion.x;
    odom_to_base_transform_msg.transform.rotation.y = quaternion.y;
    odom_to_base_transform_msg.transform.rotation.z = quaternion.z;
    odom_to_base_transform_msg.transform.rotation.w = quaternion.w;

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(odom_to_base_transform_msg);
    signals_.tf_publisher->publish(tf_msg);
}

void SimulatorNode::publishTelemetryMessage(const rclcpp::Time &time) {
    truck_msgs::msg::HardwareTelemetry telemetry_msg;
    telemetry_msg.header.frame_id = "odom_ekf";
    telemetry_msg.header.stamp = time;
    // To do: разный steering
    telemetry_msg.current_left_steering = getLeftSteering();
    telemetry_msg.current_right_steering = getRightSteering();
    telemetry_msg.target_left_steering = getTargetLeftSteering();
    telemetry_msg.target_right_steering = getTargetRightSteering();
    signals_.telemetry->publish(telemetry_msg);
}

void SimulatorNode::publishSimulationStateMessage(const rclcpp::Time &time, 
        const double speed, const geom::Angle &steering) {

    truck_msgs::msg::SimulationState state_msg;
    state_msg.header.frame_id = "odom_ekf";
    state_msg.header.stamp = time;
    state_msg.speed = speed;
    state_msg.steering = steering.radians();
    signals_.state->publish(state_msg);
}

void SimulatorNode::publishSignals() {
    engine_.advance(params_.update_period);
    const auto pose = engine_.getPose();
    const auto steering = engine_.getSteering();
    const auto linearVelocity = engine_.getLinearVelocity();
    const auto angularVelocity = engine_.getAngularVelocity();
    const auto speed = engine_.getSpeed();
    const auto time = now();
    publishOdometryMessage(time, pose, linearVelocity, angularVelocity);
    publishTransformMessage(time, pose);
    publishTelemetryMessage(time);
    publishSimulationStateMessage(time, speed, steering);
}

}  // namespace truck::simulator
