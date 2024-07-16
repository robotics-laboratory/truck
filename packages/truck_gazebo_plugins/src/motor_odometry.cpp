#include "truck_gazebo_plugins/motor_odometry.h"
#include "truck_gazebo_plugins/common.h"

#include "common/math.h"

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>

namespace gazebo {

void MotorOdometryPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    node_ = gazebo_ros::Node::Get(sdf);

    model_ =
        truck::model::makeUniquePtr(node_->get_logger(), GetParam<std::string>(sdf, "config_path"));

    left_joint_ = GetJoint(model, GetParam<std::string>(sdf, "left_joint"));
    right_joint_ = GetJoint(model, GetParam<std::string>(sdf, "right_joint"));

    period_ = std::chrono::duration<double>(GetParam<double>(sdf, "period"));

    slot_.odometry =
        node_->create_publisher<nav_msgs::msg::Odometry>("/hardware/wheel/odometry", 1);
    RCLCPP_INFO(
        node_->get_logger(), "Publish odometry on [/motor/odom] with period %f s", period_.count());

    update_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MotorOdometryPlugin::OnUpdate, this, std::placeholders::_1));
}

nav_msgs::msg::Odometry MotorOdometryPlugin::GetOdometry(const common::Time& now) const {
    nav_msgs::msg::Odometry odometry;

    odometry.header.frame_id = "base";
    odometry.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(now);
    odometry.child_frame_id = "base";

    const double left_angular_velocity = left_joint_->GetVelocity(0);
    const double right_angular_velocity = right_joint_->GetVelocity(0);

    const double angular_velocity = (left_angular_velocity + right_angular_velocity) / 2.0;
    constexpr double error = 0.001;

    odometry.twist.twist.linear.x = angular_velocity * model_->wheel().radius;
    odometry.twist.covariance[0] = truck::squared(error);

    return odometry;
}

void MotorOdometryPlugin::OnUpdate(const common::UpdateInfo& info) {
    const common::Time now = info.simTime;
    if (now - state_.last_update_time > period_.count()) {
        return;
    }

    slot_.odometry->publish(GetOdometry(now));
    state_.last_update_time = now;
}

GZ_REGISTER_MODEL_PLUGIN(MotorOdometryPlugin)

}  // namespace gazebo
