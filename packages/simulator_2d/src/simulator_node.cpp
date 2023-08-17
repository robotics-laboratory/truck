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

    signals_.tf_publisher = Node::create_publisher<tf2_msgs::msg::TFMessage>(
        "/ekf/odometry/transform", 
        rclcpp::QoS(1).reliability(qos));

    createOdometryMessage();
    createTransformMessage();

    auto model = model::makeUniquePtr(
        this->get_logger(), Node::declare_parameter<std::string>("model_config", "model.yaml"));
    engine_.start(model, this->declare_parameter("integration_step", 0.001), params_.precision);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(params_.update_period),
        std::bind(&SimulatorNode::publishSignals, this));
}

void SimulatorNode::handleControl(const truck_msgs::msg::Control::ConstSharedPtr control) {
    /*
    RCLCPP_INFO_STREAM(this->get_logger(), 
        std::to_string(control->velocity) + " " + std::to_string(control->acceleration) 
            + " " + std::to_string(control->curvature));
    //*/

    engine_.setControl(control->velocity, control->acceleration, control->curvature);
    engine_.setControl(0.1, 0, 0);
}

void SimulatorNode::createOdometryMessage() {
    msgs_.odometry.header.frame_id = "odom_ekf";
    msgs_.odometry.child_frame_id = "odom_ekf";
    msgs_.odometry.pose.pose.position.z = 0.0;
    msgs_.odometry.pose.pose.orientation.z = 0.0;
    msgs_.odometry.twist.twist.linear.z = 0.0;
    msgs_.odometry.twist.twist.angular.z = 0.0;
}

void SimulatorNode::createTransformMessage() {
    msgs_.odom_to_base_transform.header.frame_id = "odom_ekf";
    msgs_.odom_to_base_transform.child_frame_id = "base";
    msgs_.odom_to_base_transform.transform.translation.z = 0.0;
    msgs_.odom_to_base_transform.transform.rotation.z = 0.0;
}

void SimulatorNode::publishOdometryMessage(const geom::Pose &pose, const geom::Vec2 &linearVelocity, 
    const geom::Vec2 &angularVelocity) {

    msgs_.odometry.header.stamp = now();

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

void SimulatorNode::publishTelemetryMessage(const rclcpp::Time &time, const geom::Angle &steering) {
    truck_msgs::msg::HardwareTelemetry telemetry_msg;
    telemetry_msg.header.frame_id = "odom_ekf";
    telemetry_msg.header.stamp = time;
    telemetry_msg.steering_angle = steering.radians();
    signals_.telemetry->publish(telemetry_msg);
}

void SimulatorNode::publishSimulationStateMessafe(const rclcpp::Time &time, 
        const double speed, const geom::Angle &steering) {

    truck_msgs::msg::SimulationState state_msg;
    state_msg.header.frame_id = "odom_ekf";
    state_msg.header.stamp = time;
    state_msg.speed = speed;
    state_msg.steering = steering.radians();
    signals_.state->publish(state_msg);
}

void SimulatorNode::publishTransformMessage(const geom::Pose &pose) {
    msgs_.odom_to_base_transform.header.stamp = now();

    // Set the transformation.
    msgs_.odom_to_base_transform.transform.translation.x = pose.pos.x;
    msgs_.odom_to_base_transform.transform.translation.y = pose.pos.y;
    msgs_.odom_to_base_transform.transform.rotation.x = pose.dir.x;
    msgs_.odom_to_base_transform.transform.rotation.y = pose.dir.y;

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(msgs_.odom_to_base_transform);
    signals_.tf_publisher->publish(tf_msg);
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
    publishTelemetryMessage(time, steering);
    publishSimulationStateMessafe(time, speed, steering);
    auto pose = engine_.getPose();
    auto linearVelocity = engine_.getLinearVelocity();
    auto angularVelocity = engine_.getAngularVelocity();
    publishOdometryMessage(pose, linearVelocity, angularVelocity);
    publishTransformMessage(pose);
}

}  // namespace truck::simulator