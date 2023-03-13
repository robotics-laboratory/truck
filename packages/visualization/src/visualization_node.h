#pragma once

#include "model/model.h"
#include "truck_interfaces/msg/control.hpp"
#include "truck_interfaces/msg/control_mode.hpp"
#include "truck_interfaces/msg/waypoints.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>

namespace truck::visualization {

class VisualizationNode : public rclcpp::Node {
  public:
    VisualizationNode();

    struct Parameters {
        double ego_z_lev = 0.0;
        double ego_height = 0.0;

        double arc_z_lev = 0.0;
        double arc_width = 0.0;
        double arc_length = 1.0;

        double target_z_lev = 0.0;
        double target_width = 0.1;

        double waypoints_z_lev = 0.0;
        double waypoints_radius = 0.0;
    };

  private:
    void handleControl(truck_interfaces::msg::Control::ConstSharedPtr control);
    void handleMode(truck_interfaces::msg::ControlMode::ConstSharedPtr msg);
    void handleWaypoints(truck_interfaces::msg::Waypoints::ConstSharedPtr msg);
    void handleOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);

    void publishEgo() const;
    void publishArc() const;
    void publishTarget() const;
    void publishControl() const;
    void publishWaypoints(const truck_interfaces::msg::Waypoints& waypoints) const;

    std_msgs::msg::ColorRGBA velocityToColor(double speed) const;

    Parameters params_;
    model::Model model_;

    struct State {
        truck_interfaces::msg::ControlMode::ConstSharedPtr mode = nullptr;
        truck_interfaces::msg::Control::ConstSharedPtr control = nullptr;
        nav_msgs::msg::Odometry::ConstSharedPtr odom = nullptr;
    } state_;

    struct Slots {
        rclcpp::Subscription<truck_interfaces::msg::Control>::SharedPtr control = nullptr;
        rclcpp::Subscription<truck_interfaces::msg::ControlMode>::SharedPtr mode = nullptr;
        rclcpp::Subscription<truck_interfaces::msg::Waypoints>::SharedPtr waypoints = nullptr;
        // foxglove has twitching if publish ego pose in base frame, use odom for smoother result!
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
    } slot_;

    struct Signals {
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ego = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arc = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoints = nullptr;
    } signal_;
};

}  // namespace truck::control_proxy