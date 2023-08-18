#include "simulator_2d/simulator_node.h"

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

    signals_.odom = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos));

    signals_.tf_publisher = Node::create_publisher<tf2_msgs::msg::TFMessage>(
        "/ekf/odometry/transform", 
        rclcpp::QoS(1).reliability(qos));

    createOdometryMessage();
    createTransformMessage();

    auto model = model::makeUniquePtr(
        this->get_logger(),
        Node::declare_parameter<std::string>("model_config", "model.yaml"));
    engine_.start(model, this->declare_parameter("simulation_tick", 0.01),
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
        engine_.setControl(0.1, 0);
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

void SimulatorNode::publishOdometryMessage(const geom::Pose &pose, const geom::Vec2 &linearVelocity, 
    const geom::Vec2 &angularVelocity) {

    msgs_.odometry.header.stamp = now();

    // Set the position.
    msgs_.odometry.pose.pose.position.x = pose.pos.x;
    msgs_.odometry.pose.pose.position.y = pose.pos.y;
    msgs_.odometry.pose.pose.orientation.x = pose.dir.x;
    msgs_.odometry.pose.pose.orientation.y = pose.dir.y;

    // Set the velocity.
    msgs_.odometry.twist.twist.linear.x = linearVelocity.x;
    msgs_.odometry.twist.twist.linear.y = linearVelocity.y;
    msgs_.odometry.twist.twist.angular.x = angularVelocity.x;
    msgs_.odometry.twist.twist.angular.y = angularVelocity.y;

    signals_.odom->publish(msgs_.odometry);
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
    auto pose = engine_.getPose();
    auto linearVelocity = engine_.getLinearVelocity();
    auto angularVelocity = engine_.getAngularVelocity();
    publishOdometryMessage(pose, linearVelocity, angularVelocity);
    publishTransformMessage(pose);
}

} // namespace truck::simulator