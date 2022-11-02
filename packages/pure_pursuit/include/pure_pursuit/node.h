#pragma once

#include "pure_pursuit/controller.h"
#include "pure_pursuit/util.h"

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
    PurePursuitNode(): Node("PurePursuitNode") {
        const Params params = {
            .model_path = this->declare_parameter<std::string>("model_config", "model.yaml"),
            .radius =  Limits<double>{
                this->declare_parameter<double>("radius/min", 0.3),
                this->declare_parameter<double>("radius/max", 1.0),
            },
            .velocity_factor = this->declare_parameter<double>("velocity_factor", 0.2),
        };

        RCLCPP_INFO(this->get_logger(), "Model acquired...");

        controller_ = std::make_unique<Controller>(params);

        const auto qos = static_cast<rmw_qos_reliability_policy_t>(
            this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

        path_slot_ = this->create_subscription<nav_msgs::msg::Path>(
            "/motion/path",
            1,
            std::bind(&PurePursuitNode::handlePath, this, std::placeholders::_1));

        odometry_slot_ = Node::create_subscription<nav_msgs::msg::Odometry>(
            "/ekf/odometry/filtered",
            rclcpp::QoS(1).reliability(qos),
            std::bind(&PurePursuitNode::handleOdometry, this, std::placeholders::_1));

        command_signal_ =
            Node::create_publisher<truck_interfaces::msg::Control>("/motion/command", 1);
    }

  private:
    void publishCommand() {
        auto toMsg = [this](const Command& cmd) {
            truck_interfaces::msg::Control msg;

            msg.header.frame_id = "base";
            msg.header.stamp = now();

            msg.velocity = cmd.velocity;
            msg.acceleration = cmd.acceleration;
            msg.curvature = cmd.curvature;

            return msg;
        };

        if (!path_ || !odometry_) {
            command_signal_->publish(toMsg(Command::stop()));
            return;
        }

        const auto result = (*controller_)(*odometry_, *path_);
        if (!result) {
            RCLCPP_ERROR(get_logger(), toString(result.error()).data());
            command_signal_->publish(toMsg(Command::stop()));
            return;
        }

        command_signal_->publish(toMsg(*result));
   }

   void handlePath(nav_msgs::msg::Path::SharedPtr path) {
       path_ = std::move(path);
       publishCommand();
   }

    void handleOdometry(nav_msgs::msg::Odometry::SharedPtr odometry) {
        odometry_ = std::move(odometry);
        publishCommand();
    }

    nav_msgs::msg::Path::ConstSharedPtr path_ = nullptr;
    nav_msgs::msg::Odometry::ConstSharedPtr odometry_ = nullptr;

    // input
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_slot_ = nullptr;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_slot_ = nullptr;

    // output
    rclcpp::Publisher<truck_interfaces::msg::Control>::SharedPtr command_signal_ = nullptr;

    std::unique_ptr<Controller> controller_ = nullptr;
};

};  // namespace truck::pure_pursuit