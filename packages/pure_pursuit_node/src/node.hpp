#pragma once

#include "planning_interfaces/msg/path.hpp"
#include "pure_pursuit_msgs/msg/state.hpp"
#include "pure_pursuit_msgs/msg/command.hpp"

#include "rclcpp/rclcpp.hpp"

#include "controller.hpp"

#include <optional>
#include <memory>

namespace pure_pursuit {

class PursuitNode : public rclcpp::Node {
public:
    PursuitNode()
        : Node("PursuitNode")
        , controller(Parameters(*this))
    {
        slot_path = this->create_subscription<planning_interfaces::msg::Path>(
            "planned_path",
            1,
            [this](planning_interfaces::msg::Path::UniquePtr path) {
                if (path->exists)
                    trajectory = std::move(path->trajectory);
            }
        );
        slot_state = Node::create_subscription<pure_pursuit_msgs::msg::State>(
            "current_state",
            1,
            [this](pure_pursuit_msgs::msg::State::UniquePtr state) {
                if (trajectory) {
                    auto cmd = controller.get_motion(*state, *trajectory);
                    if (cmd)
                        cmd_publisher->publish(*cmd);
                }
            }
        );
    }

private:
    rclcpp::Subscription<planning_interfaces::msg::Path>::SharedPtr slot_path;
    rclcpp::Subscription<pure_pursuit_msgs::msg::State>::SharedPtr slot_state;
    rclcpp::Publisher<pure_pursuit_msgs::msg::Command>::SharedPtr cmd_publisher;
    std::optional<std::vector<planning_interfaces::msg::Point>> trajectory;

    Controller controller;
};

};