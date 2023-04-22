#pragma once

#include "model/model.h"
#include "truck_msgs/msg/control.hpp"
#include "truck_msgs/msg/control_mode.hpp"
#include "truck_msgs/msg/trajectory.hpp"
#include "truck_msgs/msg/waypoints.hpp"

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

        double waypoints_z_lev = 0.0;
        double waypoints_radius = 0.0;

        double trajectory_z_lev = 0.0;
        double trajectory_width = 0.0;
    };

  private:
    void handleTrajectory(truck_msgs::msg::Trajectory::ConstSharedPtr trajectory);
    void handleControl(truck_msgs::msg::Control::ConstSharedPtr control);
    void handleMode(truck_msgs::msg::ControlMode::ConstSharedPtr msg);
    void handleWaypoints(truck_msgs::msg::Waypoints::ConstSharedPtr msg);
    void handleOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);

    void publishTrajectory(const truck_msgs::msg::Trajectory& trajectory) const;
    void publishEgo() const;
    void publishArc() const;
    void publishControl() const;
    void publishWaypoints(const truck_msgs::msg::Waypoints& waypoints) const;

    std_msgs::msg::ColorRGBA velocityToColor(double speed, double alpha=1.0) const;

    Parameters params_;
    model::Model model_;

    struct State {
        truck_msgs::msg::ControlMode::ConstSharedPtr mode = nullptr;
        truck_msgs::msg::Control::ConstSharedPtr control = nullptr;
        nav_msgs::msg::Odometry::ConstSharedPtr odom = nullptr;
    } state_;

    struct Slots {
        rclcpp::Subscription<truck_msgs::msg::Trajectory>::SharedPtr trajectory = nullptr;
        rclcpp::Subscription<truck_msgs::msg::Control>::SharedPtr control = nullptr;
        rclcpp::Subscription<truck_msgs::msg::ControlMode>::SharedPtr mode = nullptr;
        rclcpp::Subscription<truck_msgs::msg::Waypoints>::SharedPtr waypoints = nullptr;
        // foxglove has twitching if publish ego pose in base frame, use odom for smoother result!
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
    } slot_;

    struct Signals {
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ego = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arc = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoints = nullptr;
    } signal_;
};

}  // namespace truck::control_proxy