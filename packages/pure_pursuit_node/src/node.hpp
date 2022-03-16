#pragma once

#include "planning_interfaces/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
                trajectory = std::move(path->path.poses);
            }
        );
        slot_state = Node::create_subscription<nav_msgs::msg::Odometry>(
            "current_state",
            1,
            [this](nav_msgs::msg::Odometry::UniquePtr odometry) {
                if (trajectory) {
                    auto cmd = controller.get_motion(*odometry, *trajectory);
                    cmd_publisher->publish(cmd);
                }
            }
        );
    }

private:
    rclcpp::Subscription<planning_interfaces::msg::Path>::SharedPtr slot_path;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slot_state;
    rclcpp::Publisher<pure_pursuit_msgs::msg::Command>::SharedPtr cmd_publisher;
    std::optional<std::vector<geometry_msgs::msg::PoseStamped>> trajectory;

    Controller controller;
};

};