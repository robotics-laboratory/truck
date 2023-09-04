#include "simulator_2d/simulator_node.h"

#include "geom/msg.h"

#include <cmath>

namespace truck::simulator {

using namespace std::placeholders;

SimulatorNode::SimulatorNode() : Node("simulator") {
    auto model = model::makeUniquePtr(
        this->get_logger(),
        Node::declare_parameter<std::string>("model_config", "model.yaml"));
    const auto wheel_base = model->wheelBase();

    params_ = Parameters{
        .update_period 
            = std::chrono::milliseconds(this->declare_parameter<long int>("update_period", 250)),
        .show_wheel_normals = this->declare_parameter("show_wheel_normals", false),
        .wheel_x_offset = wheel_base.base_to_rear,
        .wheel_y_offset = wheel_base.width / 2,
        .precision = this->declare_parameter("calculations_precision", 1e-8)
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

    signals_.normals = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/simulator/wheel_normals", rclcpp::QoS(1).reliability(qos));

    createOdometryMessage();
    createTransformMessage();
    createWheelNormalsMessage();

    engine_.start(model, this->declare_parameter("simulation_tick", 0.01), 
        this->declare_parameter("integration_steps", 1000), 
        params_.precision);

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
        engine_.setControl(10, 10);
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

void SimulatorNode::createWheelNormalsMessage() {
    msgs_.normals_.header.frame_id = "base";
    msgs_.normals_.id = 0;

    msgs_.normals_.type = visualization_msgs::msg::Marker::LINE_LIST;
    msgs_.normals_.action = visualization_msgs::msg::Marker::ADD;
    msgs_.normals_.lifetime = rclcpp::Duration::from_seconds(0);

    msgs_.normals_.pose.position.x = 0.0;
    msgs_.normals_.pose.position.y = 0.0;
    msgs_.normals_.pose.position.z = 0.0;

    msgs_.normals_.scale.x = 0.05;

    msgs_.normals_.color.a = 1.0;
    msgs_.normals_.color.r = 0.6;
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

void SimulatorNode::publishWheelNormalsMessage(const rclcpp::Time &time, const geom::Angle &steering) {

    msgs_.normals_.header.stamp = time;

    const double steering_rad = steering.radians();
    if (abs(steering_rad) < params_.precision) {
        msgs_.normals_.points.clear();
        signals_.normals->publish(msgs_.normals_);
        return;
    }

    msgs_.normals_.points.resize(8);
    for (auto i = 0; i < 8; ++i) {
        msgs_.normals_.points[i].x = 0;
        msgs_.normals_.points[i].y = 0;
    }

    // Front right wheel.
    msgs_.normals_.points[0].x = params_.wheel_x_offset;
    msgs_.normals_.points[0].y = -params_.wheel_y_offset;

    // Front left wheel
    msgs_.normals_.points[2].x = params_.wheel_x_offset;
    msgs_.normals_.points[2].y = params_.wheel_y_offset;

    // Rear right wheel.
    msgs_.normals_.points[4].x = -params_.wheel_x_offset;
    msgs_.normals_.points[4].y = -params_.wheel_y_offset;

    // Rear left wheel.
    msgs_.normals_.points[6].x = -params_.wheel_x_offset;
    msgs_.normals_.points[6].y = params_.wheel_y_offset;

    // Instant turning point.
    const double radius = 2 * params_.wheel_x_offset / tan(steering_rad); // Тут лучше брать честно длину машинки вместо 2*x_offset, упрощаю
    msgs_.normals_.points[1].x = -params_.wheel_x_offset;
    msgs_.normals_.points[1].y = radius;

    for (auto i = 3; i < 8; i += 2) {
        msgs_.normals_.points[i] = msgs_.normals_.points[1];
    }

    signals_.normals->publish(msgs_.normals_);
}

void SimulatorNode::publishSignals() {
    engine_.suspend();
    const auto time = now();
    const auto pose = engine_.getPose();
    const auto steering = engine_.getSteering();
    const auto linearVelocity = engine_.getLinearVelocity();
    const auto angularVelocity = engine_.getAngularVelocity();
    engine_.resume();
    if (params_.show_wheel_normals) {
        publishWheelNormalsMessage(time, steering);
    }

    publishOdometryMessage(time, pose, linearVelocity, angularVelocity);
    publishTransformMessage(time, pose);
    publishTelemetryMessage(time, steering);
}

} // namespace truck::simulator