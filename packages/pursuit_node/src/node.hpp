#pragma once

#include "planning_interfaces/msg/path.hpp"
#include "pursuit_interfaces/msg/state.hpp"
#include "pursuit_interfaces/msg/command.hpp"

#include "rclcpp/rclcpp.hpp"

#include "controller.hpp"

#include <optional>
#include <memory>

namespace pursuit {

class PursuitNode : public rclcpp::Node {
public:
    PursuitNode()
        : Node("PursuitNode")
        , controller(Parameters{
              Node::get_parameter("max_velocity").get_value<double>()
            , Node::get_parameter("max_accel").get_value<double>()
            , Node::get_parameter("lookahead_distance").get_value<double>()
        })
    {
        path_subscription = this->create_subscription<planning_interfaces::msg::Path>(
            "planned_path",
            1,
            [this](planning_interfaces::msg::Path::UniquePtr path) {
                if (path->exists)
                    trajectory = std::move(path->trajectory);
            }
        );
        state_subscribtion = Node::create_subscription<pursuit_interfaces::msg::State>(
            "current_state",
            1,
            [this](pursuit_interfaces::msg::State::UniquePtr state) {
                if (trajectory) {
                    auto cmd = controller.get_motion(*state, *trajectory);
                    if (cmd)
                        cmd_publisher->publish(*cmd);
                }
            }
        );
    }

private:
    rclcpp::Subscription<planning_interfaces::msg::Path>::SharedPtr path_subscription;
    rclcpp::Subscription<pursuit_interfaces::msg::State>::SharedPtr state_subscribtion;
    rclcpp::Publisher<pursuit_interfaces::msg::Command>::SharedPtr cmd_publisher;
    std::optional<std::vector<planning_interfaces::msg::Point>> trajectory;

    Controller controller;
};

};