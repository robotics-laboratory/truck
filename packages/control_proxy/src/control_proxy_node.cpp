#include "control_proxy_node.h"

#include <boost/preprocessor.hpp>
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

ControlMap::ControlMap(const YAML::Node& node)
    : velocity_axis(node["velocity_axis"].as<size_t>())
    , curvature_axis(node["curvature_axis"].as<size_t>())
    , off_button(node["off_button"].as<size_t>())
    , remote_button(node["remote_button"].as<size_t>())
    , auto_button(node["auto_button"].as<size_t>()) {}

ControlMap::ControlMap(const std::string& path) : ControlMap(YAML::LoadFile(path)) {}

namespace {

auto loadControlMap(rclcpp::Logger logger, const std::string& path) {
    RCLCPP_INFO(logger, "load control map: %s", path.c_str());
    return ControlMap(path);
}

}  // namespace

ControlProxyNode::ControlProxyNode()
    : Node("control_proxy")
    , model_(
        model::load(
            this->get_logger(),
            Node::declare_parameter<std::string>("model_config")))
    , control_map_(
        loadControlMap(
            this->get_logger(),
            Node::declare_parameter<std::string>("control_config")))
    , frame_id_("base") {
    RCLCPP_INFO(
        this->get_logger(),
        "max velocity: %f, min velocity: %f",
        model_.baseVelocityLimits().max,
        model_.baseVelocityLimits().min);

    RCLCPP_INFO(this->get_logger(), "max abs curvature: %f", model_.baseMaxAbsCurvature());

    joypad_timeout_ =
        std::chrono::milliseconds(this->declare_parameter<long int>("joypad_timeout", 200));

    RCLCPP_INFO(this->get_logger(), "joy pad timeout: %li ms", joypad_timeout_.count());

    control_timeout_ =
        std::chrono::milliseconds(this->declare_parameter<long int>("control_timeout", 200));

    RCLCPP_INFO(this->get_logger(), "control timeout: %li ms", control_timeout_.count());

    // input
    command_slot_ = Node::create_subscription<truck_msgs::msg::Control>(
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
    mode_signal_ = Node::create_publisher<truck_msgs::msg::ControlMode>("/control/mode", 1);
    command_signal_ = Node::create_publisher<truck_msgs::msg::Control>("/control/command", 1);
    twist_signal_ = Node::create_publisher<geometry_msgs::msg::Twist>("/control/twist", 1);

    RCLCPP_INFO(this->get_logger(), "mode: Off");
}

void ControlProxyNode::setMode(Mode mode) {
    if (mode == mode_) {
        return;
    }

    mode_ = mode;
    RCLCPP_WARN(this->get_logger(), "mode: %s", toString(mode).c_str());

    sensor_msgs::msg::JoyFeedback result;

    result.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
    result.id = 0;
    result.intensity = 0.5;

    mode_feedback_signal_->publish(result);
}

truck_msgs::msg::Control ControlProxyNode::makeControlCommand(
    const sensor_msgs::msg::Joy& command) {
    truck_msgs::msg::Control result;

    result.header.stamp = command.header.stamp;
    result.header.frame_id = frame_id_;

    result.curvature = command.axes[control_map_.curvature_axis] * model_.baseMaxAbsCurvature();

    result.velocity = [&] {
        const double ratio = command.axes[control_map_.velocity_axis];
        return ratio >= 0 ? ratio * model_.baseVelocityLimits().max
                          : -ratio * model_.baseVelocityLimits().min;
    }();

    return result;
}

geometry_msgs::msg::Twist ControlProxyNode::transformToTwist(
    const truck_msgs::msg::Control& command) const {
    geometry_msgs::msg::Twist twist;

    twist.linear.x = command.velocity;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = command.velocity * command.curvature;

    return twist;
}

void ControlProxyNode::handleJoypadCommand(sensor_msgs::msg::Joy::ConstSharedPtr joypad_command) {
    if (checkButtonPressed(joypad_command, control_map_.off_button)) {
        setMode(Mode::Off);
    } else if (checkButtonPressed(joypad_command, control_map_.remote_button)) {
        setMode(Mode::Remote);
    } else if (checkButtonPressed(joypad_command, control_map_.auto_button)) {
        setMode(Mode::Auto);
    }

    if (mode_ == Mode::Remote) {
        const auto command = makeControlCommand(*joypad_command);
        publishCommand(command);
        prev_command_ = std::make_shared<truck_msgs::msg::Control>(command);
    }

    prev_joypad_command_ = std::move(joypad_command);
}

bool ControlProxyNode::checkButtonPressed(
    sensor_msgs::msg::Joy::ConstSharedPtr joypad_command, size_t joypad_button) {
    if (!prev_joypad_command_ || !joypad_command) {
        return false;
    }
    return prev_joypad_command_->buttons[joypad_button] == 0 &&
           joypad_command->buttons[joypad_button] == 1;
}

void ControlProxyNode::publishMode() {
    truck_msgs::msg::ControlMode result;

    result.header.stamp = now();
    result.header.frame_id = frame_id_;

    result.mode = static_cast<uint8_t>(mode_);
    mode_signal_->publish(result);
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

void ControlProxyNode::watchdog() {
    auto timeout_failed = [this](const auto& msg, const auto& timeout) {
        if (!msg) {
            return true;
        }
        auto duration_ns = (now() - msg->header.stamp).nanoseconds();
        return std::chrono::nanoseconds(duration_ns) > timeout;
    };

    if (mode_ == Mode::Off) {
        publishStop();
        return;
    }

    if (mode_ != Mode::Off && timeout_failed(prev_joypad_command_, joypad_timeout_)) {
        RCLCPP_ERROR(this->get_logger(), "lost joypad, stop!");
        setMode(Mode::Off);
        prev_joypad_command_ = nullptr;
        publishStop();
        return;
    }

    if (mode_ == Mode::Auto && timeout_failed(prev_command_, control_timeout_)) {
        RCLCPP_ERROR(this->get_logger(), "lost control, stop!");
        setMode(Mode::Off);
        prev_command_ = nullptr;
        publishStop();
        return;
    }
}

void ControlProxyNode::publishCommand(const truck_msgs::msg::Control& command) {
    command_signal_->publish(command);
    twist_signal_->publish(transformToTwist(command));
}

void ControlProxyNode::forwardControlCommand(
    truck_msgs::msg::Control::ConstSharedPtr command) {
    if (mode_ == Mode::Auto) {
        publishCommand(*command);
        prev_command_ = command;
    }
}

}  // namespace truck::control_proxy
