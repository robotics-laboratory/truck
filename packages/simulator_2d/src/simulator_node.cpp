#include "simulator_2d/simulator_node.h"

#include "geom/msg.h"

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

    signals_.odometry = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos));

    signals_.tf_publisher = Node::create_publisher<tf2_msgs::msg::TFMessage>(
        "/ekf/odometry/transform", 
        rclcpp::QoS(1).reliability(qos));

    signals_.telemetry = Node::create_publisher<truck_msgs::msg::HardwareTelemetry>(
        "/hardware/telemetry", 
        rclcpp::QoS(1).reliability(qos));

    createOdometryMessage();
    createTransformMessage();

    auto model = model::makeUniquePtr(
        this->get_logger(),
        Node::declare_parameter<std::string>("model_config", "model.yaml"));
    engine_.start(model, this->declare_parameter("simulation_tick", 0.01), 
        this->declare_parameter("integration_steps", 1000), 
        this->declare_parameter("calculations_precision", 1e-8));

    timer_ = this->create_wall_timer(
        params_.update_period,
        std::bind(&SimulatorNode::publishSignals, this));
}

void SimulatorNode::handleControl(const truck_msgs::msg::Control::ConstSharedPtr control) {
    if (control->has_acceleration) {
        engine_.setControl(control->velocity, control->acceleration, control->curvature);
    }
    else {
        //engine_.setControl(control->velocity, control->curvature);
        engine_.setControl(0.5, 0);
    }
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

void SimulatorNode::createTelemetryMessage() {
    msgs_.telemetry.header.frame_id = "odom_ekf";
}

void SimulatorNode::publishOdometryMessage(const rclcpp::Time &time, const geom::Pose &pose, 
    const geom::Vec2 &linearVelocity, const geom::Vec2 &angularVelocity) {

    msgs_.odometry.header.stamp = time;

    // Set the position.
    msgs_.odometry.pose.pose.position.x = pose.pos.x;
    msgs_.odometry.pose.pose.position.y = pose.pos.y;
    const auto quaternion = truck::geom::msg::toQuaternion(pose.dir);
    msgs_.odometry.pose.pose.orientation.x = quaternion.x;
    msgs_.odometry.pose.pose.orientation.y = quaternion.y;
    msgs_.odometry.pose.pose.orientation.z = quaternion.z;
    msgs_.odometry.pose.pose.orientation.w = quaternion.w;

    // Set the velocity.
    msgs_.odometry.twist.twist.linear.x = linearVelocity.x;
    msgs_.odometry.twist.twist.linear.y = linearVelocity.y;
    msgs_.odometry.twist.twist.angular.x = angularVelocity.x;
    msgs_.odometry.twist.twist.angular.y = angularVelocity.y;

    signals_.odometry->publish(msgs_.odometry);
}

void SimulatorNode::publishTransformMessage(const rclcpp::Time &time, const geom::Pose &pose) {
    msgs_.odom_to_base_transform.header.stamp = time;

    // Set the transformation.
    msgs_.odom_to_base_transform.transform.translation.x = pose.pos.x;
    msgs_.odom_to_base_transform.transform.translation.y = pose.pos.y;
    const auto quaternion = truck::geom::msg::toQuaternion(pose.dir);
    msgs_.odom_to_base_transform.transform.rotation.x = quaternion.x;
    msgs_.odom_to_base_transform.transform.rotation.y = quaternion.y;
    msgs_.odom_to_base_transform.transform.rotation.z = quaternion.z;
    msgs_.odom_to_base_transform.transform.rotation.w = quaternion.w;

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(msgs_.odom_to_base_transform);
    signals_.tf_publisher->publish(tf_msg);
}

void SimulatorNode::publishTelemetryMessage(const rclcpp::Time &time, const geom::Angle &steering) {
    msgs_.telemetry.header.stamp = time;
    msgs_.telemetry.steering_angle = steering.radians();
    signals_.telemetry->publish(msgs_.telemetry);
}

void SimulatorNode::publishSignals() {
    const auto time = now();
    const auto pose = engine_.getPose();
    const auto linearVelocity = engine_.getLinearVelocity();
    const auto angularVelocity = engine_.getAngularVelocity();
    publishOdometryMessage(time, pose, linearVelocity, angularVelocity);
    publishTransformMessage(time, pose);
    const auto steering = engine_.getSteering();
    publishTelemetryMessage(time, steering);
}

} // namespace truck::simulator