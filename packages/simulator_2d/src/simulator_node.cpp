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
        .show_wheel_normals = this->declare_parameter<bool>("show_wheel_normals", false),
        .wheel_x_delta = shape.length / 2,
        .wheel_y_delta = shape.width / 2
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

    msgs_.normals_.points.resize(8);
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

void SimulatorNode::publishWheelNormalsMessage(const rclcpp::Time &time, const geom::Pose &pose,
    const geom::Angle &steering) {
    msgs_.normals_.header.stamp = time;

    //msgs_.normals_.pose.position.x = pose.pos.x;
    //msgs_.normals_.pose.position.y = pose.pos.y;
    const auto quaternion = truck::geom::msg::toQuaternion(pose.dir);
    /*
    msgs_.normals_.pose.orientation.x = quaternion.x;
    msgs_.normals_.pose.orientation.y = quaternion.y;
    msgs_.normals_.pose.orientation.z = quaternion.z;
    msgs_.normals_.pose.orientation.w = quaternion.w;
    //*/

    for (size_t i = 0; i < msgs_.normals_.points.size(); i += 2) {
        msgs_.normals_.points[i].x = pose.pos.x;
        msgs_.normals_.points[i].y = pose.pos.y;
    }

    const double rotation_angle = truck::geom::toAngle(quaternion).radians();
    double angle_sin = sin(rotation_angle);
    double angle_cos = cos(rotation_angle);

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
    angle_sin = sin(steering.radians());
    angle_cos = cos(steering.radians());
    const double w_x = msgs_.normals_.points[2].x 
        + params_.wheel_x_delta * angle_cos - params_.wheel_y_delta * angle_sin;
    const double w_y = msgs_.normals_.points[2].y
        + params_.wheel_x_delta * angle_sin + params_.wheel_y_delta * angle_cos;
    const double left_normal_a = w_x - msgs_.normals_.points[2].x;
    const double left_normal_b = -msgs_.normals_.points[2].y + w_y;
    const double left_normal_c = -left_normal_a * w_x - left_normal_b * w_y;
    const double right_normal_a = msgs_.normals_.points[4].x - msgs_.normals_.points[0].x;
    const double right_normal_b = -msgs_.normals_.points[0].y + msgs_.normals_.points[4].y;
    const double right_normal_c = -right_normal_a * msgs_.normals_.points[4].x 
        - right_normal_b * msgs_.normals_.points[4].y;

    const double divider = left_normal_a * right_normal_b - right_normal_a * left_normal_b;
    if (abs(divider) < 1e-8) {
        msgs_.normals_.points[1].x = pose.pos.x;
        msgs_.normals_.points[1].y = pose.pos.y;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), 
            "PARALLEL " + std::to_string(left_normal_a * right_normal_b) + " "
            + std::to_string(right_normal_a * left_normal_b));
    }
    else {
        msgs_.normals_.points[1].x 
            = (left_normal_b * right_normal_c - right_normal_b * left_normal_c) / divider;
        msgs_.normals_.points[1].y 
            = (right_normal_a * left_normal_c - left_normal_a * right_normal_c) / divider;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), 
            std::to_string(msgs_.normals_.points[1].x) + " " + std::to_string(msgs_.normals_.points[1].y));
    }

    for (size_t i = 3; i < msgs_.normals_.points.size(); i += 2) {
        msgs_.normals_.points[i] = msgs_.normals_.points[1];
    }
    
    //const double xx = msgs_.normals_.points[1].x - msgs_.normals_.points[0].x;
    //const double yy = msgs_.normals_.points[1].y - msgs_.normals_.points[0].y;
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), std::to_string(sqrt(xx*xx + yy*yy)));
    
    signals_.normals->publish(msgs_.normals_);
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
    if (params_.show_wheel_normals) {
        publishWheelNormalsMessage(time, pose, steering);
    }
}

} // namespace truck::simulator