#pragma once

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

#include <gazebo_ros/node.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <sdf/sdf.hh>

#include <memory>
#include <string>

namespace gazebo {

class OdometryPlugin : public ModelPlugin {
  public:
    OdometryPlugin() : ModelPlugin() {}

    ~OdometryPlugin() = default;

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

    nav_msgs::msg::Odometry GetOdometry(const common::Time& now) const;
    tf2_msgs::msg::TFMessage GetTransforms(const common::Time& now) const;

    void OnUpdate(const gazebo::common::UpdateInfo& update_info);

  private:
    double period_ = 0.0;
    ignition::math::Vector3d xyz_offset_{0, 0, 0};
    gazebo::physics::LinkPtr link_ = nullptr;

    common::Time last_update_time_ = {};
    event::ConnectionPtr update_ = nullptr;

    gazebo_ros::Node::SharedPtr node_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_ = nullptr;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_ = nullptr;
};

}  // namespace gazebo