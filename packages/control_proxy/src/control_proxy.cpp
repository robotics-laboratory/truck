#include "control_proxy.h"

#include <utility>

using namespace std::literals;
using std::placeholders::_1;

namespace truck::control_proxy {

ControlProxy::ControlProxy()
    : Node("control_proxy"), model_(Node::declare_parameter<std::string>("model_config")) {
    // params
    is_auto_ = this->declare_parameter<bool>("is_auto", false);

    remote_control_timeout_ =
        std::chrono::milliseconds(this->declare_parameter<long int>("remote_control_timeout", 200));

    // input
    commnad_slot_ = Node::create_subscription<truck_interfaces::msg::Control>(
        "/motion/target", 1, std::bind(&ControlProxy::forwardCommand, this, _1));

    remote_commnad_slot_ = Node::create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&ControlProxy::handleRemoteCommand, this, _1));

    // output
    if (remote_control_timeout_ > 0ms) {
        watchdog_timer_ = this->create_wall_timer(
            remote_control_timeout_, std::bind(&ControlProxy::watchdog, this));
    }

    mode_timer_ = this->create_wall_timer(200ms, std::bind(&ControlProxy::publishMode, this));

    command_feedback_signal_ =
        Node::create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/set_feedback", 1);

    command_signal_ = Node::create_publisher<truck_interfaces::msg::Control>("/control/command", 1);
    mode_signal_ = Node::create_publisher<truck_interfaces::msg::ControlMode>("/control/mode", 1);
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

static const auto kAutoFeedback = [] {
    sensor_msgs::msg::JoyFeedback msg;

    msg.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
    msg.id = 0;
    msg.intensity = 1.0;

    return msg;
}();

static const auto kRemoteFeedback = [] {
    sensor_msgs::msg::JoyFeedback msg;

    msg.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
    msg.id = 0;
    msg.intensity = 0.4;

    return msg;
}();

}  // namespace

truck_interfaces::msg::Control ControlProxy::makeCommand(
    sensor_msgs::msg::Joy::ConstSharedPtr command) {
    truck_interfaces::msg::Control result;

    result.curvature = command->axes[AXE_LX] * model_.baseMaxAbsCurvature();

    result.velocity = [&] {
        const double ratio = command->axes[AXE_RY];
        return ratio >= 0 ? ratio * model_.baseVelocityLimits().max
                          : -ratio * model_.baseVelocityLimits().min;
    }();

    return result;
}

void ControlProxy::handleRemoteCommand(sensor_msgs::msg::Joy::ConstSharedPtr command) {
    if (switchModeToRemote(prev_remote_command_, command)) {
        if (is_auto_) {
            command_feedback_signal_->publish(kRemoteFeedback);
        }

        is_auto_ = false;
    } else if (switchModeToAuto(prev_remote_command_, command)) {
        if (!is_auto_) {
            command_feedback_signal_->publish(kAutoFeedback);
        }

        is_auto_ = true;
    }

    if (!is_auto_) {
        command_signal_->publish(makeCommand(command));
    }

    prev_remote_command_ = std::move(command);
}

void ControlProxy::publishMode() {
    truck_interfaces::msg::ControlMode mode;
    mode.is_auto = is_auto_;
    mode_signal_->publish(mode);
}

void ControlProxy::watchdog() { publishMode(); }

void ControlProxy::forwardCommand(truck_interfaces::msg::Control::ConstSharedPtr command) {
    if (is_auto_) {
        command_signal_->publish(*command);
    }
}

}  // namespace truck::control_proxy