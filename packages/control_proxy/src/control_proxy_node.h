#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "common/exception.h"
#include "model/model.h"

#include "truck_msgs/msg/control.hpp"
#include "truck_msgs/msg/control_mode.hpp"
#include "truck_msgs/msg/remote_control.hpp"
#include <std_srvs/srv/empty.hpp>

#include <chrono>
#include <cstdint>
#include <string_view>

namespace truck::control_proxy {

enum class Mode : uint8_t { kOff = 0, kRemote = 1, kAuto = 2 };

// Pay attention! The returned string are null terminated, but still be careful when using it.
constexpr std::string_view toString(Mode mode) {
    switch (mode) {
        case Mode::kOff:
            return "Off";
        case Mode::kRemote:
            return "Remote";
        case Mode::kAuto:
            return "Auto";
        default:
            FALL("Unexpected mode: %d", static_cast<uint8_t>(mode));
    }
}

class ControlProxyNode : public rclcpp::Node {
  public:
    ControlProxyNode();

  private:
    truck_msgs::msg::Control makeControlCommand(const truck_msgs::msg::RemoteControl& command);

    void publishCommand(const truck_msgs::msg::Control& command);

    void forwardControlCommand(truck_msgs::msg::Control::ConstSharedPtr command);

    void handleRemoteCommand(truck_msgs::msg::RemoteControl::ConstSharedPtr command);

    void onReset(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        const std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void watchdog();
    void reset();

    void setMode(Mode mode);
    void publishMode();
    void publishStop();

    const std::string frame_id_ = "base";
    std::unique_ptr<model::Model> model_ = nullptr;

    struct Params {
        std::chrono::milliseconds watchdog_period{20};
        std::chrono::milliseconds mode_period{200};
        std::chrono::milliseconds control_timeout{150};
        std::chrono::milliseconds remote_timeout{200};
    } params_{};

    struct Services {
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset = nullptr;
    } service_;

    struct Slots {
        rclcpp::Subscription<truck_msgs::msg::Control>::SharedPtr command = nullptr;
        rclcpp::Subscription<truck_msgs::msg::RemoteControl>::SharedPtr input = nullptr;
    } slot_;

    struct Signals {
        rclcpp::Publisher<truck_msgs::msg::Control>::SharedPtr command = nullptr;
        rclcpp::Publisher<truck_msgs::msg::ControlMode>::SharedPtr mode = nullptr;
    } signal_;

    struct Timers {
        rclcpp::TimerBase::SharedPtr watchdog = nullptr;
        rclcpp::TimerBase::SharedPtr mode = nullptr;
    } timer_;

    struct State {
        Mode mode = Mode::kOff;
        truck_msgs::msg::RemoteControl::ConstSharedPtr prev_input = nullptr;
        truck_msgs::msg::Control::ConstSharedPtr prev_command = nullptr;
    } state_;
};

}  // namespace truck::control_proxy
