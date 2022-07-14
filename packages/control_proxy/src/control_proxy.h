#pragma once

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include <rclcpp/rclcpp.hpp>

#include "model/model.h"

#include "truck_interfaces/msg/control.hpp"
#include "truck_interfaces/msg/control_mode.hpp"

#include <chrono>
#include <string>

namespace truck::control_proxy {

class ControlProxy : public rclcpp::Node {
  public:
    ControlProxy();

  private:
    truck_interfaces::msg::Control makeCommand(sensor_msgs::msg::Joy::ConstSharedPtr command);
    void forwardCommand(truck_interfaces::msg::Control::ConstSharedPtr command);
    void handleRemoteCommand(sensor_msgs::msg::Joy::ConstSharedPtr command);
    void watchdog();

    void publishMode();

    model::Model model_;

    // params
    std::chrono::milliseconds remote_control_timeout_{200};

    // input
    rclcpp::Subscription<truck_interfaces::msg::Control>::SharedPtr commnad_slot_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr remote_commnad_slot_ = nullptr;

    // output
    rclcpp::TimerBase::SharedPtr watchdog_timer_ = nullptr;
    rclcpp::TimerBase::SharedPtr mode_timer_ = nullptr;
    rclcpp::Publisher<truck_interfaces::msg::Control>::SharedPtr command_signal_ = nullptr;
    rclcpp::Publisher<truck_interfaces::msg::ControlMode>::SharedPtr mode_signal_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr command_feedback_signal_ = nullptr;

    // state
    bool is_auto_ = false;
    sensor_msgs::msg::Joy::ConstSharedPtr prev_remote_command_ = nullptr;
};

}  // namespace truck::control_proxy