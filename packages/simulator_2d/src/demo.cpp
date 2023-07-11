#include "simulator_2d/demo.h"

DemoNode::DemoNode() : Node("demo") {
    //*
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    publisher_ = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/demo/publisher",
        rclcpp::QoS(1).reliability(qos));

    timer_ = this->create_wall_timer(period_, std::bind(&DemoNode::timerCallback, this));
    //*/
}

void DemoNode::timerCallback() {
    sphere_.header.stamp = now();
    sphere_.header.frame_id = "odom_ekf";
    sphere_.id = 0;

    sphere_.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_.action = visualization_msgs::msg::Marker::ADD;
    sphere_.lifetime = rclcpp::Duration::from_seconds(0);

    sphere_.pose.position.x = 0.0;
    sphere_.pose.position.y = 0.0;
    sphere_.pose.position.z = 0.0;

    sphere_.scale.x = 1.0;
    sphere_.scale.y = 1.0;
    sphere_.scale.z = 1.0;

    sphere_.color.a = 1.0;
    sphere_.color.r = 1.0;
    sphere_.color.g = 0.0;
    sphere_.color.b = 0.0;

    publisher_->publish(sphere_);
}