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
            = this->declare_parameter("params_.update_period", 0.01),
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

    engine_.start(model, this->declare_parameter("integration_step", 0.001), params_.precision);

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

void SimulatorNode::publishOdometryMessage(const rclcpp::Time &time, const geom::Pose &pose, 
    const geom::Vec2 &linearVelocity, const geom::Vec2 &angularVelocity) {

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

void SimulatorNode::publishTelemetryMessage(const rclcpp::Time &time, const geom::Angle &steering) {
    truck_msgs::msg::HardwareTelemetry telemetry_msg;
    telemetry_msg.header.frame_id = "odom_ekf";
    telemetry_msg.header.stamp = time;
    telemetry_msg.steering_angle = steering.radians();
    signals_.telemetry->publish(telemetry_msg);
}

void SimulatorNode::publishWheelNormalsMessage(const rclcpp::Time &time, const geom::Angle &steering) {
    visualization_msgs::msg::Marker normals_msg;
    normals_msg.header.frame_id = "base";
    normals_msg.id = 0;
    normals_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
    normals_msg.action = visualization_msgs::msg::Marker::ADD;
    normals_msg.lifetime = rclcpp::Duration::from_seconds(0);
    normals_msg.scale.x = 0.05;
    normals_msg.color.a = 1.0;
    normals_msg.color.r = 0.6;
    normals_msg.header.stamp = time;

    const double steering_rad = steering.radians();
    if (abs(steering_rad) < params_.precision) {
        normals_msg.points.clear();
        signals_.normals->publish(normals_msg);
        return;
    }

    normals_msg.points.resize(8);
    for (auto i = 0; i < 8; ++i) {
        normals_msg.points[i].x = 0;
        normals_msg.points[i].y = 0;
    }

    // Front right wheel.
    normals_msg.points[0].x = params_.wheel_x_offset;
    normals_msg.points[0].y = -params_.wheel_y_offset;

    // Front left wheel
    normals_msg.points[2].x = params_.wheel_x_offset;
    normals_msg.points[2].y = params_.wheel_y_offset;

    // Rear right wheel.
    normals_msg.points[4].x = -params_.wheel_x_offset;
    normals_msg.points[4].y = -params_.wheel_y_offset;

    // Rear left wheel.
    normals_msg.points[6].x = -params_.wheel_x_offset;
    normals_msg.points[6].y = params_.wheel_y_offset;

    // Instant turning point.
    const double radius = 2 * params_.wheel_x_offset / tan(steering_rad); // Тут лучше брать честно длину машинки вместо 2*x_offset, упрощаю
    normals_msg.points[1].x = -params_.wheel_x_offset;
    normals_msg.points[1].y = radius;

    for (auto i = 3; i < 8; i += 2) {
        normals_msg.points[i] = normals_msg.points[1];
    }

    signals_.normals->publish(normals_msg);
}

void SimulatorNode::publishSignals() {
    engine_.advance(params_.update_period);
    const auto pose = engine_.getPose();
    const auto steering = engine_.getSteering();
    const auto linearVelocity = engine_.getLinearVelocity();
    const auto angularVelocity = engine_.getAngularVelocity();
    const auto time = now();
    if (params_.show_wheel_normals) {
        publishWheelNormalsMessage(time, steering);
    }

    publishOdometryMessage(time, pose, linearVelocity, angularVelocity);
    publishTransformMessage(time, pose);
    publishTelemetryMessage(time, steering);
}

}  // namespace truck::simulator