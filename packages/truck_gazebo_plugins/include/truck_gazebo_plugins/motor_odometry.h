#pragma once

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

#include <gazebo_ros/node.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <sdf/sdf.hh>

#include "model/model.h"

#include <chrono>
#include <memory>
#include <string>

namespace gazebo {

class MotorOdometryPlugin : public ModelPlugin {
  public:
    MotorOdometryPlugin() : ModelPlugin() {}

    ~MotorOdometryPlugin() = default;

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

    nav_msgs::msg::Odometry GetOdometry(const common::Time& now) const;

    void OnUpdate(const gazebo::common::UpdateInfo& update_info);

  private:
    event::ConnectionPtr update_ = nullptr;
    gazebo_ros::Node::SharedPtr node_ = nullptr;

    struct State {
        common::Time last_update_time = {};
    } state_;

    struct Slots {
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry = nullptr;
    } slot_;

    std::chrono::duration<double> period_{0.02};
    std::unique_ptr<truck::model::Model> model_ = nullptr;

    physics::JointPtr left_joint_ = nullptr;
    physics::JointPtr right_joint_ = nullptr;
};

}  // namespace gazebo
