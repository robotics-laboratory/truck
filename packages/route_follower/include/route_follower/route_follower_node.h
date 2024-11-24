#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "truck_msgs/msg/navigation_route.hpp"
#include "motion/trajectory.h"
#include "speed/greedy_planner.h"
#include "collision/collision_checker.h"
#include "geom/localization.h"

#include <rclcpp/time.hpp>
#include <tf2_ros/qos.hpp>

#include <chrono>
#include <optional>
#include <memory>

namespace truck::route_follower {

using namespace std::chrono_literals;
using namespace std::placeholders;

class RouteFollowerNode : public rclcpp::Node {
  public:
    RouteFollowerNode();

  private:
    void publishTrajectory(motion::Trajectory& trajectory);
    void onRoute(const truck_msgs::msg::NavigationRoute::SharedPtr msg);

    speed::GreedyPlanner::Params speed_params_{};

    struct Slots {
        rclcpp::Subscription<truck_msgs::msg::NavigationRoute>::SharedPtr route = nullptr;
    } slot_;

    struct Signals {
        rclcpp::Publisher<truck_msgs::msg::Trajectory>::SharedPtr trajectory = nullptr;
    } signal_;

    struct Parameters {
        std::chrono::duration<double> period = 0.1s;
        double safety_margin = 0.3;
    } params_;

    struct State {
        nav_msgs::msg::Odometry::SharedPtr odometry = nullptr;
        std::optional<geom::Localization> localization = std::nullopt;
        nav_msgs::msg::OccupancyGrid::SharedPtr grid = nullptr;
        std::shared_ptr<collision::Map> distance_transform = nullptr;
        double scheduled_velocity = 0;
    } state_;

    std::unique_ptr<model::Model> model_ = nullptr;
};

}  // namespace truck::route_follower