#include "truck_gazebo_plugins/odometry.h"

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>

namespace gazebo {

namespace {

void setCovariance(std::array<double, 36>& cov, double eps) {
    const double sq = eps * eps;
    cov[0] = sq;
    cov[7] = sq;
    cov[14] = sq;
    cov[21] = sq;
    cov[28] = sq;
    cov[35] = sq;
}

}  // namespace

void OdometryPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    // world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);

    std::string link_name = "";

    period_ = std::chrono::milliseconds(sdf->Get<size_t>("period", 50).first);
    link_name = sdf->Get<std::string>("link_name");
    xyz_offset_ = sdf->Get<ignition::math::Vector3d>("xyz_offset", xyz_offset_).first;

    link_ = model->GetLink(link_name);

    if (!link_) {
        RCLCPP_ERROR(node_->get_logger(), "Link '%s' does not exist!", link_name.c_str());
        return;
    }

    odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/simulation/odom", 1);
    tf_publisher_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/simulation/tf", 1);

    RCLCPP_INFO(node_->get_logger(), "Publish odometry on [/odom] with period %zu ms", period_.count());

    update_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&OdometryPlugin::OnUpdate, this, std::placeholders::_1));
}

nav_msgs::msg::Odometry OdometryPlugin::GetOdometry(const common::Time& now) const {
    nav_msgs::msg::Odometry odometry;

    odometry.header.frame_id = "odom";
    odometry.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(now);
    odometry.child_frame_id = "odom";

    ignition::math::Pose3d pose = link_->WorldPose();
    pose.Pos() += xyz_offset_;

    odometry.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(link_->WorldPose());

    odometry.twist.twist.linear =
        gazebo_ros::Convert<geometry_msgs::msg::Vector3>(link_->WorldLinearVel());

    odometry.twist.twist.angular =
        gazebo_ros::Convert<geometry_msgs::msg::Vector3>(link_->WorldAngularVel());

    static const double eps = 1e-6;

    setCovariance(odometry.pose.covariance, eps);
    setCovariance(odometry.twist.covariance, eps);

    return odometry;
}

tf2_msgs::msg::TFMessage OdometryPlugin::GetTransforms(const common::Time& now) const {
    tf2_msgs::msg::TFMessage result;

    {  // world -> odom
        geometry_msgs::msg::TransformStamped world_to_odom;

        world_to_odom.header.frame_id = "world";
        world_to_odom.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(now);
        world_to_odom.child_frame_id = "odom";

        world_to_odom.transform.translation.x = 0;
        world_to_odom.transform.translation.y = 0;
        world_to_odom.transform.translation.z = 0;

        world_to_odom.transform.rotation.x = 0;
        world_to_odom.transform.rotation.y = 0;
        world_to_odom.transform.rotation.z = 0;
        world_to_odom.transform.rotation.w = 1;

        result.transforms.push_back(world_to_odom);
    }

    {  // odom -> base
        geometry_msgs::msg::TransformStamped odom_to_base;

        ignition::math::Pose3d pose = link_->WorldPose();
        pose.Pos() += xyz_offset_;

        odom_to_base.header.frame_id = "odom";
        odom_to_base.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(now);
        odom_to_base.child_frame_id = "base";

        odom_to_base.transform.translation =
            gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose.Pos());

        odom_to_base.transform.rotation =
            gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

        result.transforms.push_back(odom_to_base);
    }

    return result;
}

void OdometryPlugin::OnUpdate(const common::UpdateInfo& info) {
    const common::Time now = info.simTime;
    if (now - last_update_time_ > period_.count()) {
        return;
    }

    odometry_publisher_->publish(GetOdometry(now));
    tf_publisher_->publish(GetTransforms(now));
    last_update_time_ = now;
}

GZ_REGISTER_MODEL_PLUGIN(OdometryPlugin)

}  // namespace gazebo