#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "model/model.h"

#include "truck_interfaces/msg/control.hpp"
#include "truck_interfaces/msg/control_mode.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace truck::control_proxy {

enum class Mode : uint8_t { Off = 0, Remote = 1, Auto = 2 };

std::string toString(Mode mode);

struct ControlMap {
    ControlMap(const YAML::Node& node);
    ControlMap(const std::string& path);

    static constexpr size_t none = -1;

    const size_t velocity_axis = none;
    const size_t curvature_axis = none;

    const size_t off_button = none;
    const size_t remote_button = none;
    const size_t auto_button = none;
};

class ControlProxyNode : public rclcpp::Node {
  public:
    ControlProxyNode();

  private:
    truck_interfaces::msg::Control makeControlCommand(const sensor_msgs::msg::Joy& joypad_command);

    void publishCommand(const truck_interfaces::msg::Control& command);

    geometry_msgs::msg::Twist transformToTwist(const truck_interfaces::msg::Control& command) const;

    void forwardControlCommand(truck_interfaces::msg::Control::ConstSharedPtr command);

    void handleJoypadCommand(sensor_msgs::msg::Joy::ConstSharedPtr joypad_command);

    bool checkButtonPressed(
        sensor_msgs::msg::Joy::ConstSharedPtr joypad_command, size_t joypad_button);

    void watchdog();

    void setMode(Mode mode);
    void publishMode();
    void publishStop();

    const model::Model model_;
    const ControlMap control_map_;
    const std::string frame_id_;

    // params
    std::chrono::milliseconds control_timeout_{200};
    std::chrono::milliseconds joypad_timeout_{200};

    // input
    rclcpp::Subscription<truck_interfaces::msg::Control>::SharedPtr command_slot_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joypad_slot_ = nullptr;

    // output
    rclcpp::TimerBase::SharedPtr watchdog_timer_ = nullptr;
    rclcpp::TimerBase::SharedPtr publish_mode_timer_ = nullptr;
    rclcpp::Publisher<truck_interfaces::msg::Control>::SharedPtr command_signal_ = nullptr;
    rclcpp::Publisher<truck_interfaces::msg::ControlMode>::SharedPtr mode_signal_ = nullptr;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_signal_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr mode_feedback_signal_ = nullptr;

    // state
    Mode mode_ = Mode::Off;
    sensor_msgs::msg::Joy::ConstSharedPtr prev_joypad_command_ = nullptr;
    truck_interfaces::msg::Control::ConstSharedPtr prev_command_ = nullptr;
};

}  // namespace truck::control_proxy