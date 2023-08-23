#include "simulator_2d/simulator_node.h"

#include "geom/msg.h"

#include <cmath>

namespace truck::simulator {

using namespace std::placeholders;

SimulatorNode::SimulatorNode() : Node("simulator") {
    auto model = model::makeUniquePtr(
        this->get_logger(),
        Node::declare_parameter<std::string>("model_config", "model.yaml"));
    const auto shape = model->shape();

    params_ = Parameters{
        .update_period 
            = std::chrono::milliseconds(this->declare_parameter<long int>("update_period", 250)),
        .show_wheel_normals = this->declare_parameter("show_wheel_normals", false),
        .wheel_x_delta = shape.length / 2,
        .wheel_y_delta = shape.width / 2,
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
        std::chrono::duration<double>(params_.update_period),
        std::bind(&SimulatorNode::publishSignals, this));
}

void SimulatorNode::handleControl(const truck_msgs::msg::Control::ConstSharedPtr control) {
    if (control->has_acceleration) {
        engine_.setControl(control->velocity, control->acceleration, control->curvature);
    }
    else {
        engine_.setControl(control->velocity, control->curvature);
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
    msgs_.normals_.header.frame_id = "odom_ekf";
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

void SimulatorNode::publishWheelNormalsMessage(const rclcpp::Time &time, 
    const geom::Pose &pose, const geom::Angle &steering) {

    msgs_.normals_.header.stamp = time;

    const double steering_rad = steering.radians();
    if (abs(steering_rad) < params_.precision) {
        msgs_.normals_.points.clear();
        signals_.normals->publish(msgs_.normals_);
        return;
    }

    msgs_.normals_.points.resize(8);
    for (auto i = 0; i < 8; i += 2) {
        msgs_.normals_.points[i].x = pose.pos.x;
        msgs_.normals_.points[i].y = pose.pos.y;
    }

    const double rotation_angle 
        = truck::geom::toAngle(truck::geom::msg::toQuaternion(pose.dir)).radians();
    const double angle_sin = sin(rotation_angle);
    const double angle_cos = cos(rotation_angle);

    // Front right wheel.
    msgs_.normals_.points[0].x 
        += params_.wheel_x_delta * angle_cos + params_.wheel_y_delta * angle_sin;
    msgs_.normals_.points[0].y 
        += params_.wheel_x_delta * angle_sin - params_.wheel_y_delta * angle_cos;

    // Front left wheel
    msgs_.normals_.points[2].x 
        += params_.wheel_x_delta * angle_cos - params_.wheel_y_delta * angle_sin;
    msgs_.normals_.points[2].y 
        += params_.wheel_x_delta * angle_sin + params_.wheel_y_delta * angle_cos;

    // Rear right wheel.
    msgs_.normals_.points[4].x 
        += -params_.wheel_x_delta * angle_cos + params_.wheel_y_delta * angle_sin;
    msgs_.normals_.points[4].y 
        += -params_.wheel_x_delta * angle_sin - params_.wheel_y_delta * angle_cos;

    // Rear left wheel.
    msgs_.normals_.points[6].x 
        += -params_.wheel_x_delta * angle_cos - params_.wheel_y_delta * angle_sin;
    msgs_.normals_.points[6].y 
        += -params_.wheel_x_delta * angle_sin + params_.wheel_y_delta * angle_cos;

    // Instant turning point.
    const double w_x = msgs_.normals_.points[2].x 
        + params_.wheel_x_delta * cos(rotation_angle + steering_rad);
    const double w_y = msgs_.normals_.points[2].y 
        + params_.wheel_x_delta * sin(rotation_angle + steering_rad);

    const double normal_a_a = w_x - msgs_.normals_.points[2].x;
    const double normal_a_b = w_y - msgs_.normals_.points[2].y;
    const double normal_a_c = -normal_a_a * msgs_.normals_.points[2].x 
        - normal_a_b * msgs_.normals_.points[2].y;
    const double normal_b_a = msgs_.normals_.points[4].x - msgs_.normals_.points[0].x;
    const double normal_b_b = -msgs_.normals_.points[0].y + msgs_.normals_.points[4].y;
    const double normal_b_c = -normal_b_a * msgs_.normals_.points[4].x 
        - normal_b_b * msgs_.normals_.points[4].y;

    const double divider = normal_a_a * normal_b_b - normal_b_a * normal_a_b;
    if (abs(divider) < params_.precision) {
        msgs_.normals_.points.clear();
    }
    else {
        msgs_.normals_.points[1].x 
            = (normal_a_b * normal_b_c - normal_b_b * normal_a_c) / divider;
        msgs_.normals_.points[1].y 
            = (normal_b_a * normal_a_c - normal_a_a * normal_b_c) / divider;
        for (auto i = 3; i < 8; i += 2) {
            msgs_.normals_.points[i] = msgs_.normals_.points[1];
        }
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
        publishWheelNormalsMessage(time, pose, steering);
    }

    publishOdometryMessage(time, pose, linearVelocity, angularVelocity);
    publishTransformMessage(time, pose);
    publishTelemetryMessage(time, steering);
}

}  // namespace truck::simulator