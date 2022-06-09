#pragma once

#include <memory>
#include <optional>
#include <string>
#include <limits>

#include "pure_pursuit/controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "truck_interfaces/msg/path.hpp"
#include "truck_interfaces/msg/control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pure_pursuit/visual_info.hpp"
#include "model/model.hpp"

namespace pure_pursuit {

class PursuitNode : public rclcpp::Node {
  private:
    static inline truck_interfaces::msg::Control getStopCommand() {
        truck_interfaces::msg::Control stop;
        stop.velocity = 0;
        stop.acceleration = -std::numeric_limits<double>::infinity();
        stop.curvature = 0;
        return stop;
    }

  public:
    PursuitNode()
        : Node("PursuitNode")
        , controller(
              model::Model(Node::declare_parameter<std::string>("model_config_path")),
              ControllerConfig(Node::declare_parameter<std::string>("controller_config_path"))) {
        bool publish_debug_info = this->declare_parameter<bool>("publish_debug_info", true);

        cmd_publisher =
            Node::create_publisher<truck_interfaces::msg::Control>("pure_pursuit_command", 1);
        if (publish_debug_info) {
            arc_publisher =
                Node::create_publisher<visualization_msgs::msg::MarkerArray>("pure_pursuit_arc", 1);
        }
        slot_path = this->create_subscription<truck_interfaces::msg::Path>(
            "planned_path", 1, [this](truck_interfaces::msg::Path::UniquePtr path) {
                trajectory = std::move(path->path.poses);
            });
        slot_state = Node::create_subscription<nav_msgs::msg::Odometry>(
            "current_state",
            1,
            [this, publish_debug_info](nav_msgs::msg::Odometry::UniquePtr odometry) {
                const auto STOP = getStopCommand();
                if (!trajectory) {
                    cmd_publisher->publish(STOP);
                    return;
                }
                ControllerResult res =
                    controller.getMotion(*odometry, *trajectory, publish_debug_info);
                if (!res) {
                    RCLCPP_ERROR(get_logger(), errorToString(res.error()).c_str());
                    cmd_publisher->publish(STOP);
                    return;
                }
                cmd_publisher->publish(res->cmd);
                if (publish_debug_info) {
                    arc_publisher->publish(res->visual_info->arc);
                }
            });
    }

  private:
    rclcpp::Subscription<truck_interfaces::msg::Path>::SharedPtr slot_path;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slot_state;
    rclcpp::Publisher<truck_interfaces::msg::Control>::SharedPtr cmd_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arc_publisher;
    std::optional<std::vector<geometry_msgs::msg::PoseStamped>> trajectory;

    Controller controller;
};

};  // namespace pure_pursuit