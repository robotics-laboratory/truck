#pragma once

#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "model/model.h"

#include "truck_msgs/msg/control.hpp"
#include "truck_msgs/msg/control_mode.hpp"
#include "truck_msgs/msg/remote_control.hpp"
#include <std_srvs/srv/empty.hpp>

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
    truck_msgs::msg::Control makeControlCommand(const truck_msgs::msg::RemoteControl& command);

    void publishCommand(const truck_msgs::msg::Control& command);

    geometry_msgs::msg::Twist transformToTwist(const truck_msgs::msg::Control& command) const;

    void forwardControlCommand(truck_msgs::msg::Control::ConstSharedPtr command);

    void handleInputCommand(truck_msgs::msg::RemoteControl::ConstSharedPtr command);

    void onReset(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>);

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
        std::chrono::milliseconds control_timeout{200};
        std::chrono::milliseconds input_timeout{200};
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
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist = nullptr;
    } signal_;

    struct Timers {
        rclcpp::TimerBase::SharedPtr watchdog = nullptr;
        rclcpp::TimerBase::SharedPtr mode = nullptr;
    } timer_;
 
    struct State {
        Mode mode = Mode::Off;
        truck_msgs::msg::RemoteControl::ConstSharedPtr prev_input = nullptr;
        truck_msgs::msg::Control::ConstSharedPtr prev_command = nullptr;
    } state_;
};

}  // namespace truck::control_proxy