#pragma once

#include <boost/preprocessor.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include "model/model.h"

#include "truck_interfaces/msg/control.hpp"
#include "truck_interfaces/msg/control_mode.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace truck::control_proxy {

enum class Mode : uint8_t { Off = 0, Remote = 1, Auto = 2 };

std::string toString(Mode mode);

class ControlProxyNode : public rclcpp::Node {
  public:
    ControlProxyNode();

  private:
    truck_interfaces::msg::Control makeControlCommand(
        sensor_msgs::msg::Joy::ConstSharedPtr joypad_command);

    geometry_msgs::msg::TwistStamped turnControlToTwist(
        truck_interfaces::msg::Control command);

    void forwardControlCommand(truck_interfaces::msg::Control::ConstSharedPtr command);

    void handleJoypadCommand(sensor_msgs::msg::Joy::ConstSharedPtr joypad_command);

    void watchdog();

    void setMode(Mode mode);
    void publishMode();
    void publishStop();

    model::Model model_;

    // params
    std::chrono::milliseconds control_timeout_{200};
    std::chrono::milliseconds joypad_timeout_{200};
    std::string frame_id_;

    // input
    rclcpp::Subscription<truck_interfaces::msg::Control>::SharedPtr command_slot_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joypad_slot_ = nullptr;

    // output
    rclcpp::TimerBase::SharedPtr watchdog_timer_ = nullptr;
    rclcpp::TimerBase::SharedPtr publish_mode_timer_ = nullptr;
    rclcpp::Publisher<truck_interfaces::msg::Control>::SharedPtr command_signal_ = nullptr;
    rclcpp::Publisher<truck_interfaces::msg::ControlMode>::SharedPtr mode_signal_ = nullptr;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_signal_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr mode_feedback_signal_ = nullptr;

    // state
    Mode mode_ = Mode::Off;
    sensor_msgs::msg::Joy::ConstSharedPtr prev_joypad_command_ = nullptr;
    truck_interfaces::msg::Control::ConstSharedPtr prev_command_ = nullptr;
};

}  // namespace truck::control_proxy