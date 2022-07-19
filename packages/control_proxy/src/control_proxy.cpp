#include "control_proxy.h"

#include <boost/assert.hpp>
#include <rclcpp/time.hpp>

#include <utility>

using namespace std::literals;
using std::placeholders::_1;

namespace truck::control_proxy {

std::string toString(Mode mode) {
    switch(mode) {
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

ControlProxy::ControlProxy()
    : Node("control_proxy"), model_(Node::declare_parameter<std::string>("model_config")) {
    joypad_timeout_ =
        std::chrono::milliseconds(this->declare_parameter<long int>("joypad_timeout", 200));

    RCLCPP_INFO(this->get_logger(), "joy pad timeout %li ms", joypad_timeout_.count());

    control_timeout_ =
        std::chrono::milliseconds(this->declare_parameter<long int>("control_timeout", 200));

    RCLCPP_INFO(this->get_logger(), "control timeout %li ms", control_timeout_.count());

    // input
    commnad_slot_ = Node::create_subscription<truck_interfaces::msg::Control>(
        "/motion/command", 1, std::bind(&ControlProxy::forwardControlCommand, this, _1));

    joypad_slot_ = Node::create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&ControlProxy::handleJoypadCommand, this, _1));

    // output
    const auto watchdog_period = std::min(control_timeout_, joypad_timeout_);
    BOOST_ASSERT(watchdog_period > 0ms);

    watchdog_timer_ = this->create_wall_timer(
        watchdog_period, std::bind(&ControlProxy::watchdog, this));

    publish_mode_timer_ = this->create_wall_timer(200ms, std::bind(&ControlProxy::publishMode, this));

    mode_feedback_signal_ =
        Node::create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/set_feedback", 1);

    command_signal_ = Node::create_publisher<truck_interfaces::msg::Control>("/control/command", 1);
    mode_signal_ = Node::create_publisher<truck_interfaces::msg::ControlMode>("/control/mode", 1);

    RCLCPP_INFO(this->get_logger(), "Mode: Off");
}

namespace {

constexpr size_t BUTTON_CROSS = 0;
constexpr size_t BUTTON_CIRCLE = 1;

constexpr size_t AXE_LX = 0;
constexpr size_t AXE_LY = 1;

constexpr size_t AXE_RX = 3;
constexpr size_t AXE_RY = 4;

bool switchModeToRemote(
    const sensor_msgs::msg::Joy::ConstSharedPtr& previous,
    const sensor_msgs::msg::Joy::ConstSharedPtr& current) {
    if (!previous || !current) {
        return false;
    }

    return previous->buttons[BUTTON_CROSS] == 0 && current->buttons[BUTTON_CROSS] == 1;
}

bool switchModeToAuto(
    const sensor_msgs::msg::Joy::ConstSharedPtr& previous,
    const sensor_msgs::msg::Joy::ConstSharedPtr& current) {
    if (!previous || !current) {
        return false;
    }

    return previous->buttons[BUTTON_CIRCLE] == 1 && current->buttons[BUTTON_CIRCLE] == 0;
}

} // namespace

void ControlProxy::setMode(Mode mode) {
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

truck_interfaces::msg::Control ControlProxy::makeControlCommand(
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

void ControlProxy::handleJoypadCommand(sensor_msgs::msg::Joy::ConstSharedPtr joypad_command) {
    if (switchModeToRemote(prev_joypad_command_, joypad_command)) {
        setMode(Mode::Remote);
    } else if (switchModeToAuto(prev_joypad_command_, joypad_command)) {
        setMode(Mode::Auto);
    }

    if (mode_ == Mode::Remote) {
        const auto command = makeControlCommand(joypad_command);
        command_signal_->publish(command);
        prev_command_ = std::make_shared<truck_interfaces::msg::Control>(command);
    }

    prev_joypad_command_ = std::move(joypad_command); 
}

void ControlProxy::publishMode() {
    truck_interfaces::msg::ControlMode result;

    result.header.stamp = now();
    result.header.frame_id = "base";

    result.mode = static_cast<uint8_t>(mode_);
    mode_signal_->publish(result);
}

void ControlProxy::publishStop() {
    static const auto stop = [this] {
        truck_interfaces::msg::Control result;

        result.header.stamp = now();
        result.header.frame_id = "base";

        result.velocity = 0.0;
        result.curvature =  0.0;
        result.acceleration = 0.0;

        return result;
    }();

    command_signal_->publish(stop);
}

void ControlProxy::watchdog() {
    auto get_delay = [this](const auto& msg) {
        return std::chrono::nanoseconds((now() - msg.header.stamp).nanoseconds());
    };

    if (!prev_joypad_command_ || get_delay(*prev_joypad_command_) > joypad_timeout_) {
        if (mode_ != Mode::Off) {
            RCLCPP_ERROR(this->get_logger(), "Lose joypad, stop!");
            setMode(Mode::Off);
        }

        prev_joypad_command_ = nullptr;
        publishStop();
        return;
    }

    setMode(std::max(Mode::Remote, mode_));

    if (!prev_command_ || get_delay(*prev_command_) > control_timeout_) {
        if (mode_ != Mode::Remote) {
            RCLCPP_ERROR(this->get_logger(), "Lose control, stop!");
            setMode(Mode::Remote);
        }

        prev_command_ = nullptr;
        publishStop();
        return;
    }
}

void ControlProxy::forwardControlCommand(truck_interfaces::msg::Control::ConstSharedPtr command) {
    if (mode_ == Mode::Auto) {
        command_signal_->publish(*command);
        prev_command_ = command;
    }
}

}  // namespace truck::control_proxy