#pragma once

#include "model/model.h"
#include "truck_msgs/msg/control.hpp"
#include "truck_msgs/msg/control_mode.hpp"
#include "truck_msgs/msg/trajectory.hpp"
#include "truck_msgs/msg/waypoints.hpp"
#include "truck_msgs/msg/hardware_telemetry.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

namespace truck::visualization {

class VisualizationNode : public rclcpp::Node {
  public:
    VisualizationNode();

  private:
    void initializeParams();
    void initializeTopicHandlers();
    void initializeCacheWheelBaseTfs();
    void initializeCache();

    void handleTrajectory(truck_msgs::msg::Trajectory::ConstSharedPtr trajectory);
    void handleControl(truck_msgs::msg::Control::ConstSharedPtr control);
    void handleMode(truck_msgs::msg::ControlMode::ConstSharedPtr msg);
    void handleWaypoints(truck_msgs::msg::Waypoints::ConstSharedPtr msg);
    void handleTelemetry(truck_msgs::msg::HardwareTelemetry::ConstSharedPtr msg);
    void handleOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);

    void publishTrajectory() const;
    void publishEgo() const;
    void publishEgoTrack() const;
    void publishArc() const;
    void publishWaypoints() const;

    std_msgs::msg::ColorRGBA velocityToColor(double speed, double alpha=1.0) const;

    struct Parameters {
        rclcpp::Duration ttl = rclcpp::Duration::from_seconds(1.0);

        double ego_z_lev = 0.0;
        double ego_height = 0.0;

        double ego_track_width = 0.06;
        double ego_track_height = 0.01;
        rclcpp::Duration ego_track_ttl = rclcpp::Duration::from_seconds(2.0);
        size_t ego_track_rate = 5;

        double arc_z_lev = 0.0;
        double arc_width = 0.0;
        double arc_length = 1.0;

        double waypoints_z_lev = 0.0;
        double waypoints_radius = 0.0;

        double trajectory_z_lev = 0.0;
        double trajectory_width = 0.0;
    } params_{};

    enum WheelIndex { 
        front_left = 0, 
        front_right = 1, 
        rear_left = 2,
        rear_right = 3 
    };

    struct Cache {
        geometry_msgs::msg::Vector3 ego_body_scale;
        geometry_msgs::msg::Vector3 ego_wheel_scale;
        std::array<tf2::Transform, 4> wheel_base_tfs;
        std::array<int, 4> all_wheels {
            WheelIndex::front_left,
            WheelIndex::front_right,
            WheelIndex::rear_left,
            WheelIndex::rear_right
        };
    } cache_;

    std::unique_ptr<model::Model> model_ = nullptr;

    struct State {
        size_t odom_seq_id = 0;
        truck_msgs::msg::ControlMode::ConstSharedPtr mode = nullptr;
        truck_msgs::msg::Control::ConstSharedPtr control = nullptr;
        nav_msgs::msg::Odometry::ConstSharedPtr odom = nullptr;
        truck_msgs::msg::Trajectory::ConstSharedPtr trajectory = nullptr;
        truck_msgs::msg::HardwareTelemetry::ConstSharedPtr telemetry = nullptr;
        truck_msgs::msg::Waypoints::ConstSharedPtr waypoints = nullptr;
    } state_;

    struct Slots {
        rclcpp::Subscription<truck_msgs::msg::Trajectory>::SharedPtr trajectory = nullptr;
        rclcpp::Subscription<truck_msgs::msg::Control>::SharedPtr control = nullptr;
        rclcpp::Subscription<truck_msgs::msg::ControlMode>::SharedPtr mode = nullptr;
        rclcpp::Subscription<truck_msgs::msg::Waypoints>::SharedPtr waypoints = nullptr;
        rclcpp::Subscription<truck_msgs::msg::HardwareTelemetry>::SharedPtr telemetry = nullptr;
        // foxglove has twitching if publish ego pose in base frame, use odom for smoother result!
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
    } slot_;

    struct Signals {
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ego = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ego_track = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arc = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoints = nullptr;
    } signal_;
};

}  // namespace truck::visualization