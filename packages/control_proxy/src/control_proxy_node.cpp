#include "control_proxy_node.h"

#include "common/exception.h"
#include "common/math.h"

#include <rclcpp/time.hpp>

#include <utility>

using namespace std::literals;
using namespace std::placeholders;

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
            VERIFY(false);
    }
}

ControlProxyNode::ControlProxyNode() : Node("control_proxy") {
    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    RCLCPP_INFO(
        this->get_logger(),
        "velocity: [%f, %f] m/s",
        model_->baseVelocityLimits().max,
        model_->baseVelocityLimits().min);

    RCLCPP_INFO(this->get_logger(), "curvature: %f", model_->baseMaxAbsCurvature());

    params_ = Params {
        std::chrono::milliseconds(this->declare_parameter<long int>("watchdog_period", 20)),
        std::chrono::milliseconds(this->declare_parameter<long int>("mode_period", 200)),
        std::chrono::milliseconds(this->declare_parameter<long int>("input_timeout", 200)),
        std::chrono::milliseconds(this->declare_parameter<long int>("control_timeout", 150))
    };

    RCLCPP_INFO(this->get_logger(), "input timeout: %li ms", params_.input_timeout.count());
    RCLCPP_INFO(this->get_logger(), "control timeout: %li ms", params_.control_timeout.count());

    service_.reset = this->create_service<std_srvs::srv::Empty>(
        "/control/reset", std::bind(&ControlProxyNode::onReset, this, _1, _2));

    slot_.command = Node::create_subscription<truck_msgs::msg::Control>(
        "/motion/command", 1, std::bind(&ControlProxyNode::forwardControlCommand, this, _1));

    slot_.input = Node::create_subscription<truck_msgs::msg::RemoteControl>(
        "/control/input", 1, std::bind(&ControlProxyNode::handleInputCommand, this, _1));

    timer_.watchdog =
        this->create_wall_timer(params_.watchdog_period, std::bind(&ControlProxyNode::watchdog, this));

    timer_.mode = this->create_wall_timer(
        params_.mode_period, std::bind(&ControlProxyNode::publishMode, this));

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
}

truck_msgs::msg::Control ControlProxyNode::makeControlCommand(
    const truck_msgs::msg::RemoteControl& command) {
    truck_msgs::msg::Control result;

    result.header.stamp = command.header.stamp;
    result.header.frame_id = frame_id_;

    double rel_curvature = clamp(command.rel_curvature, -1.0, 1.0);
    double rel_velocity = clamp(command.rel_velocity, -1.0, 1.0);
    result.curvature = rel_curvature * model_->baseMaxAbsCurvature();
    result.velocity = (
        rel_velocity >= 0
        ? rel_velocity * model_->baseVelocityLimits().max
        : -rel_velocity * model_->baseVelocityLimits().min
    );
    result.has_acceleration = false;

    return result;
}

void ControlProxyNode::handleInputCommand(truck_msgs::msg::RemoteControl::ConstSharedPtr input) {
    if (state_.prev_input && state_.prev_input->mode != input->mode) {
        switch (input->mode) {
            case 0: setMode(Mode::Off); break;
            case 1: setMode(Mode::Remote); break;
            case 2: setMode(Mode::Auto); break;
            default:
                RCLCPP_WARN(get_logger(), "unknown mode: %d", input->mode);
                setMode(Mode::Off);
        }
    }

    if (state_.mode == Mode::Remote) {
        const auto command = makeControlCommand(*input);
        publishCommand(command);
        state_.prev_command = std::make_shared<truck_msgs::msg::Control>(command);
    }

    state_.prev_input = std::move(input);
}

void ControlProxyNode::publishMode() {
    truck_msgs::msg::ControlMode result;

    result.header.stamp = now();
    result.header.frame_id = frame_id_;

    result.mode = static_cast<uint8_t>(state_.mode);
    signal_.mode->publish(result);
}

void ControlProxyNode::publishStop() {
    static const auto stop = [this] {
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
    setMode(Mode::Off);
    state_.prev_input = nullptr;
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

    if (state_.mode == Mode::Off) {
        publishStop();
        return;
    }

    if (state_.mode != Mode::Off && timeout_failed(state_.prev_input, params_.input_timeout)) {
        RCLCPP_ERROR(this->get_logger(), "lost input, stop!");
        reset();
        return;
    }

    if (state_.mode == Mode::Auto && timeout_failed(state_.prev_command, params_.control_timeout)) {
        RCLCPP_ERROR(this->get_logger(), "lost control, stop!");
        reset();
        return;
    }
}

void ControlProxyNode::publishCommand(const truck_msgs::msg::Control& command) {
    signal_.command->publish(command);
}

void ControlProxyNode::forwardControlCommand(
    truck_msgs::msg::Control::ConstSharedPtr command) {
    if (state_.mode == Mode::Auto) {
        publishCommand(*command);
        state_.prev_command = command;
    }
}

void ControlProxyNode::onReset(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
    RCLCPP_WARN(this->get_logger(), "Reset!");
    reset();
}

}  // namespace truck::control_proxy
