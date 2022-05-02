#pragma once

#include <memory>
#include <optional>
#include <string>

#include "controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "truck_interfaces/msg/path.hpp"
#include "pure_pursuit_msgs/msg/command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visual_info.hpp"
#include "model.hpp"

namespace pure_pursuit {

class PursuitNode: public rclcpp::Node {
public:
    PursuitNode() : Node("PursuitNode"), controller(model::Model(Node::declare_parameter<std::string>("model_config_path"))) {
        bool publish_debug_info = this->declare_parameter<bool>("publish_debug_info", true);

        cmd_publisher =
            Node::create_publisher<pure_pursuit_msgs::msg::Command>("pure_pursuit_command", 1);
        if (publish_debug_info) {
            arc_publisher =
                Node::create_publisher<visualization_msgs::msg::MarkerArray>("pure_pursuit_arc", 1);
        }
        slot_path = this->create_subscription<truck_interfaces::msg::Path>(
            "planned_path", 1, [this](truck_interfaces::msg::Path::UniquePtr path) {
                trajectory = std::move(path->path.poses);
            });
        slot_state = Node::create_subscription<nav_msgs::msg::Odometry>(
            "current_state", 1,
            [this, publish_debug_info](nav_msgs::msg::Odometry::UniquePtr odometry) {
                if (trajectory) {
                    std::optional<ControllerResult> cmd;
                    if (publish_debug_info) {
                        VisualInfo info;
                        cmd = controller.get_motion(*odometry, *trajectory, &info);
                        arc_publisher->publish(info.arc);
                    } else {
                        cmd = controller.get_motion(*odometry, *trajectory, nullptr);
                    }
                    if (cmd) {
                        cmd_publisher->publish(**cmd);
                    } else {
                        RCLCPP_ERROR(get_logger(), cmd->get_error());
                        cmd_publisher->publish(pure_pursuit_msgs::msg::Command());
                    }
                } else {
                    cmd_publisher->publish(pure_pursuit_msgs::msg::Command());
                }
            });
    }

private:
    rclcpp::Subscription<truck_interfaces::msg::Path>::SharedPtr slot_path;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slot_state;
    rclcpp::Publisher<pure_pursuit_msgs::msg::Command>::SharedPtr cmd_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arc_publisher;
    std::optional<std::vector<geometry_msgs::msg::PoseStamped>> trajectory;

    Controller controller;
};

};  // namespace pure_pursuit