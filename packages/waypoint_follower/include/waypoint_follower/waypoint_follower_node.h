#pragma once

#include "waypoint_follower/waypoint_follower.h"

#include "collision/collision_checker.h"
#include "geom/localization.h"
#include "model/model.h"
#include "speed/greedy_planner.h"
#include "truck_msgs/msg/trajectory.hpp"
#include "truck_msgs/msg/waypoints.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <chrono>
#include <memory>
#include <optional>

namespace truck::waypoint_follower {

using namespace std::chrono_literals;

class WaypointFollowerNode : public rclcpp::Node {
  public:
    WaypointFollowerNode();

  private:
    void onWaypoint(geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onOdometry(nav_msgs::msg::Odometry::SharedPtr msg);
    void onGrid(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void onTf(tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);

    void onReset(
        const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);

    void publishTrajectory();
    void publishGridCostMap();
    void publishWaypoints();
    void publishFullState();

    std::optional<geom::Transform> getLatestTranform(
        const std::string& source, const std::string& target);

    struct Parameters {
        std::chrono::duration<double> period = 0.1s;
        double safety_margin = 0.3;
    } params_{};

    speed::GreedyPlanner::Params speed_params_{};

    struct Timers {
        rclcpp::TimerBase::SharedPtr main = nullptr;
    } timer_;

    struct Services {
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset = nullptr;
    } service_;

    struct Slots {
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint = nullptr;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid = nullptr;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf = nullptr;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static = nullptr;
    } slot_;

    struct Signals {
        rclcpp::Publisher<truck_msgs::msg::Waypoints>::SharedPtr waypoints = nullptr;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr distance_transform = nullptr;
        rclcpp::Publisher<truck_msgs::msg::Trajectory>::SharedPtr trajectory = nullptr;
    } signal_;

    struct State {
        nav_msgs::msg::Odometry::SharedPtr odometry = nullptr;
        std::optional<geom::Localization> localization = std::nullopt;
        nav_msgs::msg::OccupancyGrid::SharedPtr grid = nullptr;
        std::shared_ptr<collision::Map> distance_transform = nullptr;
        double scheduled_velocity = 0;
    } state_;

    std::unique_ptr<model::Model> model_ = nullptr;
    std::unique_ptr<WaypointFollower> follower_ = nullptr;
    std::unique_ptr<collision::StaticCollisionChecker> checker_ = nullptr;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
};

}  // namespace truck::waypoint_follower