#pragma once

#include <memory>
#include <optional>
#include <string>
#include <limits>

#include "controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "truck_interfaces/msg/path.hpp"
#include "pure_pursuit_msgs/msg/command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visual_info.hpp"
#include "model/model.hpp"

namespace pure_pursuit {

class PursuitNode: public rclcpp::Node {
private:
    static inline auto get_stop_command() {
        pure_pursuit_msgs::msg::Command stop;
        stop.velocity = 0;
        stop.acceleration = -std::numeric_limits<double>::infinity();
        stop.curvature = 0;
        return stop;
    }

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
                const auto STOP = get_stop_command();
                if (trajectory) {
                    ControllerResult cmd;
                    if (publish_debug_info) {
                        VisualInfo info;
                        cmd = controller.get_motion(*odometry, *trajectory, &info);
                        arc_publisher->publish(info.arc);
                    } else {
                        cmd = controller.get_motion(*odometry, *trajectory, nullptr);
                    }
                    if (cmd) {
                        cmd_publisher->publish(*cmd);
                    } else {
                        RCLCPP_ERROR(get_logger(), cmd.error());
                        cmd_publisher->publish(STOP);
                    }
                } else {
                    cmd_publisher->publish(STOP);
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