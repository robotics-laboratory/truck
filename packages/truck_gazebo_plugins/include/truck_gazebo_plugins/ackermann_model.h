#pragma once

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

#include <gazebo_ros/node.hpp>

#include "common/chrono.h"
#include "model/model.h"
#include "truck_interfaces/msg/control.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace gazebo {

using namespace std::literals;

class AckermannModelPlugin : public ModelPlugin {
  public:
    AckermannModelPlugin() : ModelPlugin(){};

    ~AckermannModelPlugin() = default;

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

    void OnUpdate(const gazebo::common::UpdateInfo& update_info);

    void Reset() override;

  protected:
  private:
    std::mutex mutex_;

    std::unique_ptr<truck::model::Model> model_ = nullptr;

    event::ConnectionPtr update_ = nullptr;

    double steering_error_ = 0.0;
    double steering_torque_ = 0.0;

    physics::JointPtr steering_left_joint_ = nullptr;
    physics::JointPtr steering_right_joint_ = nullptr;


    common::PID velocity_left_pd_{};
    common::PID velocity_right_pd_{};
    physics::JointPtr rear_left_joint_ = nullptr;
    physics::JointPtr rear_right_joint_ = nullptr;

    rclcpp::Duration timeout_{std::chrono::milliseconds(100)};
    common::Time last_update_time_{};
    bool emergency_stop_ = false;

    gazebo_ros::Node::SharedPtr node_ = nullptr;
    truck_interfaces::msg::Control::ConstSharedPtr command_ = nullptr;
    rclcpp::Subscription<truck_interfaces::msg::Control>::SharedPtr command_slot_ = nullptr;
};

}  // namespace gazebo