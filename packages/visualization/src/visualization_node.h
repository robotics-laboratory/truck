#pragma once

#include "model/model.h"
#include "truck_interfaces/msg/control.hpp"
#include "truck_interfaces/msg/control_mode.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>

namespace truck::visualization {

class VisualizationNode : public rclcpp::Node {
  public:
    VisualizationNode();

  private:
    void handleOdometry(const nav_msgs::msg::Odometry& msg);

    void handleMode(truck_interfaces::msg::ControlMode::ConstSharedPtr msg);

    model::Model model_;

    truck_interfaces::msg::ControlMode::ConstSharedPtr mode_ = nullptr;

    // input
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_slot_ = nullptr;
    rclcpp::Subscription<truck_interfaces::msg::ControlMode>::SharedPtr mode_slot_ = nullptr;

    // output
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ego_signal_ = nullptr;
};

}  // namespace truck::control_proxy