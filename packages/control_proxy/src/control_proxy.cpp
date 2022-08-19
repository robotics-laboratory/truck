#include "control_proxy.h"

#include <boost/assert.hpp>
#include <rclcpp/time.hpp>

#include <utility>

using namespace std::literals;
using std::placeholders::_1;

namespace truck::control_proxy {

std::string toString(Mode mode) {
    switch (mode) {
        case Mode::Off:
            return "Off";
        case Mode::Remote:
            return "Remote";
        case Mode::Auto:
            return "Auto";
        default:
            BOOST_ASSERT(false);
    }
}

ControlProxyNode::ControlProxyNode()
    : Node("control_proxy"), model_(Node::declare_parameter<std::string>("model_config")) {
    joypad_timeout_ =
        std::chrono::milliseconds(this->declare_parameter<long int>("joypad_timeout", 200));

    RCLCPP_INFO(this->get_logger(), "joy pad timeout %li ms", joypad_timeout_.count());

    control_timeout_ =
        std::chrono::milliseconds(this->declare_parameter<long int>("control_timeout", 200));

    RCLCPP_INFO(this->get_logger(), "control timeout %li ms", control_timeout_.count());

    // input
    commnad_slot_ = Node::create_subscription<truck_interfaces::msg::Control>(
        "/motion/command", 1, std::bind(&ControlProxyNode::forwardControlCommand, this, _1));

    joypad_slot_ = Node::create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&ControlProxyNode::handleJoypadCommand, this, _1));

    // output
    const auto watchdog_period = std::min(control_timeout_, joypad_timeout_);
    BOOST_ASSERT(watchdog_period > 0ms);

    watchdog_timer_ =
        this->create_wall_timer(watchdog_period, std::bind(&ControlProxyNode::watchdog, this));

    publish_mode_timer_ =
        this->create_wall_timer(200ms, std::bind(&ControlProxyNode::publishMode, this));

    mode_feedback_signal_ =
        Node::create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/set_feedback", 1);

    command_signal_ = Node::create_publisher<truck_interfaces::msg::Control>("/control/command", 1);
    mode_signal_ = Node::create_publisher<truck_interfaces::msg::ControlMode>("/control/mode", 1);

    RCLCPP_INFO(this->get_logger(), "Mode: Off");
}

namespace {

constexpr size_t BUTTON_CROSS = 0;
constexpr size_t BUTTON_CIRCLE = 1;
constexpr size_t BUTTON_TRIANGLE = 2;

constexpr size_t AXE_LX = 0;
constexpr size_t AXE_LY = 1;

constexpr size_t AXE_RX = 3;
constexpr size_t AXE_RY = 4;

}  // namespace

void ControlProxyNode::setMode(Mode mode) {
    if (mode == mode_) {
        return;
    }

    mode_ = mode;
    RCLCPP_WARN(this->get_logger(), "Mode: %s", toString(mode).c_str());

    sensor_msgs::msg::JoyFeedback result;

    result.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
    result.id = 0;
    result.intensity = 0.5;

    mode_feedback_signal_->publish(result);
}

truck_interfaces::msg::Control ControlProxyNode::makeControlCommand(
    sensor_msgs::msg::Joy::ConstSharedPtr command) {
    truck_interfaces::msg::Control result;

    result.header.stamp = now();
    result.header.frame_id = "base";

    result.curvature = command->axes[AXE_LX] * model_.baseMaxAbsCurvature();

    result.velocity = [&] {
        const double ratio = command->axes[AXE_RY];
        return ratio >= 0 ? ratio * model_.baseVelocityLimits().max
                          : -ratio * model_.baseVelocityLimits().min;
    }();

    return result;
}

void ControlProxyNode::handleJoypadCommand(sensor_msgs::msg::Joy::ConstSharedPtr joypad_command) {
    if (checkButtonPressed(joypad_command, BUTTON_CROSS)) {
        setMode(Mode::Off);
    } else if (checkButtonPressed(joypad_command, BUTTON_CIRCLE)) {
        setMode(Mode::Remote);
    } else if (checkButtonPressed(joypad_command, BUTTON_TRIANGLE)) {
        setMode(Mode::Auto);
    }

    if (mode_ == Mode::Remote) {
        const auto command = makeControlCommand(joypad_command);
        command_signal_->publish(command);
        prev_command_ = std::make_shared<truck_interfaces::msg::Control>(command);
    }

    prev_joypad_command_ = std::move(joypad_command);
}

bool ControlProxyNode::checkButtonPressed( sensor_msgs::msg::Joy::ConstSharedPtr joypad_command, size_t joypad_button) {
    if (!prev_joypad_command_ || !joypad_command) return false;
    return prev_joypad_command_->buttons[joypad_button] == 0 &&
           joypad_command->buttons[joypad_button] == 1;
}

void ControlProxyNode::publishMode() {
    truck_interfaces::msg::ControlMode result;

    result.header.stamp = now();
    result.header.frame_id = "base";

    result.mode = static_cast<uint8_t>(mode_);
    mode_signal_->publish(result);
}

void ControlProxyNode::publishStop() {
    static const auto stop = [this] {
        truck_interfaces::msg::Control result;

        result.header.stamp = now();
        result.header.frame_id = "base";

        result.velocity = 0.0;
        result.curvature = 0.0;
        result.acceleration = 0.0;

        return result;
    }();

    command_signal_->publish(stop);
}

void ControlProxyNode::watchdog() {
    auto get_delay = [this](const auto& msg) {
        return std::chrono::nanoseconds((now() - msg.header.stamp).nanoseconds());
    };

    if (mode_ == Mode::Remote && get_delay(*prev_joypad_command_) > joypad_timeout_) {
        RCLCPP_ERROR(this->get_logger(), "Lost joypad, stop!");
        setMode(Mode::Off);
        prev_joypad_command_ = nullptr;
        publishStop();
        return;
    }

    if (mode_ == Mode::Auto && get_delay(*prev_command_) > control_timeout_) {
        RCLCPP_ERROR(this->get_logger(), "Lost control, stop!");
        setMode(Mode::Remote);
        prev_command_ = nullptr;
        publishStop();
        return;
    }
}

void ControlProxyNode::forwardControlCommand(
    truck_interfaces::msg::Control::ConstSharedPtr command) {
    if (mode_ == Mode::Auto) {
        command_signal_->publish(*command);
        prev_command_ = command;
    }
}

}  // namespace truck::control_proxy