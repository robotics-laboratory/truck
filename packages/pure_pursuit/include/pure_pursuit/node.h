#pragma once

#include "pure_pursuit/controller.h"

#include "geom/msg.h"
#include "model/model.h"
#include "truck_interfaces/msg/control.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <functional>
#include <memory>
#include <string>

namespace truck::pure_pursuit {

class PurePursuitNode : public rclcpp::Node {
  public:
    PurePursuitNode() : Node("PurePursuitNode") {
        const auto model =
            model::load(this->get_logger(), Node::declare_parameter<std::string>("model_config"));

        const Params params = {
            .radius = Limits<double>{
                this->declare_parameter<double>("radius/min", 0.15),
                this->declare_parameter<double>("radius/max", 0.5),
            },
            .velocity = this->declare_parameter<double>("velocity", 0.4),
            .velocity_factor = this->declare_parameter<double>("velocity_factor", 0.2),
            .tolerance = this->declare_parameter<double>("tolerance", 0.1)
        };

        RCLCPP_INFO(this->get_logger(), "radius [%f, %f]", params.radius.min, params.radius.max);
        RCLCPP_INFO(this->get_logger(), "velocity %f", params.velocity);
        RCLCPP_INFO(this->get_logger(), "velocity factor %f", params.velocity_factor);
        RCLCPP_INFO(this->get_logger(), "tolerance %f", params.tolerance);

        controller_ = std::make_unique<Controller>(params, model);

        const auto qos = static_cast<rmw_qos_reliability_policy_t>(
            this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

        slot_.path = this->create_subscription<nav_msgs::msg::Path>(
            "/motion/path",
            1,
            std::bind(&PurePursuitNode::handlePath, this, std::placeholders::_1));

        slot_.odometry = Node::create_subscription<nav_msgs::msg::Odometry>(
            "/ekf/odometry/filtered",
            rclcpp::QoS(1).reliability(qos),
            std::bind(&PurePursuitNode::handleOdometry, this, std::placeholders::_1));

        signal_.command =
            Node::create_publisher<truck_interfaces::msg::Control>("/motion/command", 1);
    }

  private:
    void publishCommand() {
        auto toMsg = [this](const Command& cmd) {
            truck_interfaces::msg::Control msg;

            msg.header.frame_id = "base";
            msg.header.stamp = now();

            msg.velocity = cmd.velocity;
            msg.curvature = cmd.curvature;


            msg.has_target = cmd.target.has_value();
            if (cmd.target) {
                msg.target.header = state_.odometry->header;
                msg.target.point = geom::msg::toPoint(*cmd.target);
            }

            return msg;
        };

        if (!state_.path || !state_.odometry) {
            signal_.command->publish(toMsg(Command::stop()));
            return;
        }

        const auto result = (*controller_)(*state_.odometry, *state_.path);
        if (!result) {
            RCLCPP_ERROR(get_logger(), "%s", toString(result.error()).data());
            signal_.command->publish(toMsg(Command::stop()));
            return;
        }

        signal_.command->publish(toMsg(*result));
    }

    void handlePath(nav_msgs::msg::Path::SharedPtr path) {
        state_.path = std::move(path);
        publishCommand();
    }

    void handleOdometry(nav_msgs::msg::Odometry::SharedPtr odometry) {
        state_.odometry = std::move(odometry);
        publishCommand();
    }

    struct State {
        nav_msgs::msg::Path::ConstSharedPtr path = nullptr;
        nav_msgs::msg::Odometry::ConstSharedPtr odometry = nullptr;
    } state_;

    struct Slots {
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path = nullptr;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry = nullptr;
    } slot_;

    struct Signals {
        rclcpp::Publisher<truck_interfaces::msg::Control>::SharedPtr command = nullptr;
    } signal_;

    std::unique_ptr<Controller> controller_ = nullptr;
};

}  // namespace truck::pure_pursuit