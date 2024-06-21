#include "control_proxy_node.h"

#include "common/exception.h"

#include <rclcpp/time.hpp>

#include <utility>

using namespace std::literals;
using namespace std::placeholders;

namespace truck::control_proxy {

std::string toString(Mode mode) {
    switch (mode) {
        case Mode::kOff:
            return "Off";
        case Mode::kRemote:
            return "Remote";
        case Mode::kAuto:
            return "Auto";
        default:
            VERIFY(false);
    }
}

ControlMap::ControlMap(const YAML::Node& node) :
    velocity_axis(node["velocity_axis"].as<size_t>()),
    curvature_axis(node["curvature_axis"].as<size_t>()),
    off_button(node["off_button"].as<size_t>()),
    remote_button(node["remote_button"].as<size_t>()),
    auto_button(node["auto_button"].as<size_t>()) {}

ControlMap::ControlMap(const std::string& path) : ControlMap(YAML::LoadFile(path)) {}

namespace {

auto loadControlMap(const rclcpp::Logger& logger, const std::string& path) {
    RCLCPP_INFO(logger, "load control map: %s", path.c_str());
    return ControlMap(path);
}

}  // namespace

ControlProxyNode::ControlProxyNode() : Node("control_proxy") {
    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    control_map_ = std::make_unique<ControlMap>(
        loadControlMap(this->get_logger(), Node::declare_parameter<std::string>("control_config")));

    RCLCPP_INFO(
        this->get_logger(),
        "velocity: [%f, %f] m/s",
        model_->baseVelocityLimits().max,
        model_->baseVelocityLimits().min);

    RCLCPP_INFO(this->get_logger(), "curvature: %f", model_->baseMaxAbsCurvature());

    params_ = Params{
        std::chrono::milliseconds(this->declare_parameter<long int>("watchdog_period", 20)),
        std::chrono::milliseconds(this->declare_parameter<long int>("mode_period", 200)),
        std::chrono::milliseconds(this->declare_parameter<long int>("joypad_timeout", 200)),
        std::chrono::milliseconds(this->declare_parameter<long int>("control_timeout", 150))};

    RCLCPP_INFO(this->get_logger(), "joypad timeout: %li ms", params_.joypad_timeout.count());
    RCLCPP_INFO(this->get_logger(), "control timeout: %li ms", params_.control_timeout.count());

    service_.reset = this->create_service<std_srvs::srv::Empty>(
        "/control/reset", std::bind(&ControlProxyNode::onReset, this, _1, _2));

    slot_.command = Node::create_subscription<truck_msgs::msg::Control>(
        "/motion/command", 1, std::bind(&ControlProxyNode::forwardControlCommand, this, _1));

    slot_.joypad = Node::create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&ControlProxyNode::handleJoypadCommand, this, _1));

    timer_.watchdog = this->create_wall_timer(
        params_.watchdog_period, std::bind(&ControlProxyNode::watchdog, this));

    timer_.mode = this->create_wall_timer(
        params_.mode_period, std::bind(&ControlProxyNode::publishMode, this));

    signal_.mode_feedback =
        Node::create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/set_feedback", 1);
    signal_.mode = Node::create_publisher<truck_msgs::msg::ControlMode>("/control/mode", 1);
    signal_.command = Node::create_publisher<truck_msgs::msg::Control>("/control/command", 1);

    RCLCPP_INFO(this->get_logger(), "mode: Off");
}

void ControlProxyNode::setMode(Mode mode) {
    if (mode == state_.mode) {
        return;
    }

    state_.mode = mode;
    RCLCPP_WARN(this->get_logger(), "mode: %s", toString(mode).c_str());

    sensor_msgs::msg::JoyFeedback result;

    result.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
    result.id = 0;
    result.intensity = 0.5;

    signal_.mode_feedback->publish(result);
}

truck_msgs::msg::Control ControlProxyNode::makeControlCommand(
    const sensor_msgs::msg::Joy& command) {
    truck_msgs::msg::Control result;

    result.header.stamp = command.header.stamp;
    result.header.frame_id = frame_id_;

    result.curvature = command.axes[control_map_->curvature_axis] * model_->baseMaxAbsCurvature();

    result.velocity = [&] {
        const double ratio = command.axes[control_map_->velocity_axis];
        return ratio >= 0 ? ratio * model_->baseVelocityLimits().max
                          : -ratio * model_->baseVelocityLimits().min;
    }();

    result.has_acceleration = false;

    return result;
}

void ControlProxyNode::handleJoypadCommand(sensor_msgs::msg::Joy::ConstSharedPtr joypad_command) {
    if (checkButtonPressed(joypad_command, control_map_->off_button)) {
        setMode(Mode::kOff);
    } else if (checkButtonPressed(joypad_command, control_map_->remote_button)) {
        setMode(Mode::kRemote);
    } else if (checkButtonPressed(joypad_command, control_map_->auto_button)) {
        setMode(Mode::kAuto);
    }

    if (state_.mode == Mode::kRemote) {
        const auto command = makeControlCommand(*joypad_command);
        publishCommand(command);
        state_.prev_command = std::make_shared<truck_msgs::msg::Control>(command);
    }

    state_.prev_joypad_command = std::move(joypad_command);
}

bool ControlProxyNode::checkButtonPressed(
    sensor_msgs::msg::Joy::ConstSharedPtr joypad_command, size_t joypad_button) {
    if (!state_.prev_joypad_command || !joypad_command) {
        return false;
    }
    return state_.prev_joypad_command->buttons[joypad_button] == 0
           && joypad_command->buttons[joypad_button] == 1;
}

void ControlProxyNode::publishMode() {
    truck_msgs::msg::ControlMode result;

    result.header.stamp = now();
    result.header.frame_id = frame_id_;

    result.mode = static_cast<uint8_t>(state_.mode);
    signal_.mode->publish(result);
}

void ControlProxyNode::publishStop() {
    const auto stop = [this] {
        truck_msgs::msg::Control result;

        result.header.stamp = now();
        result.header.frame_id = frame_id_;

        result.velocity = 0.0;
        result.curvature = 0.0;

        return result;
    }();

    publishCommand(stop);
}

void ControlProxyNode::reset() {
    setMode(Mode::kOff);
    state_.prev_joypad_command = nullptr;
    state_.prev_command = nullptr;
    publishMode();
    publishStop();
}

void ControlProxyNode::watchdog() {
    auto timeout_failed = [this](const auto& msg, const auto& timeout) {
        if (!msg) {
            return true;
        }
        auto duration_ns = (now() - msg->header.stamp).nanoseconds();
        return std::chrono::nanoseconds(duration_ns) > timeout;
    };

    if (state_.mode == Mode::kOff) {
        publishStop();
        return;
    }

    if (state_.mode != Mode::kOff
        && timeout_failed(state_.prev_joypad_command, params_.joypad_timeout)) {
        RCLCPP_ERROR(this->get_logger(), "lost joypad, stop!");
        reset();
        return;
    }

    if (state_.mode == Mode::kAuto
        && timeout_failed(state_.prev_command, params_.control_timeout)) {
        RCLCPP_ERROR(this->get_logger(), "lost control, stop!");
        reset();
        return;
    }
}

void ControlProxyNode::publishCommand(const truck_msgs::msg::Control& command) {
    signal_.command->publish(command);
}

void ControlProxyNode::forwardControlCommand(truck_msgs::msg::Control::ConstSharedPtr command) {
    if (state_.mode == Mode::kAuto) {
        publishCommand(*command);
        state_.prev_command = command;
    }
}

void ControlProxyNode::onReset(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    RCLCPP_WARN(this->get_logger(), "Reset!");
    reset();
}

}  // namespace truck::control_proxy
