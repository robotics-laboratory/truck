#pragma once

#include "pure_pursuit/pure_pursuit.h"

#include "geom/msg.h"
#include "model/model.h"
#include "truck_msgs/msg/control.hpp"
#include "truck_msgs/msg/trajectory.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace truck::pure_pursuit {

class PurePursuitNode : public rclcpp::Node {
  public:
    PurePursuitNode();

  private:
    void publishCommand();

    void handleTrajectory(truck_msgs::msg::Trajectory::SharedPtr trajectory);

    void handleOdometry(nav_msgs::msg::Odometry::SharedPtr odometry);

    rclcpp::TimerBase::SharedPtr timer_ = nullptr;

    struct State {
        nav_msgs::msg::Odometry::SharedPtr localization_msg = nullptr;
        truck_msgs::msg::Trajectory::SharedPtr trajectory_msg = nullptr;

        std::optional<motion::Trajectory> trajectory = std::nullopt;
        std::optional<geom::Localization> localization = std::nullopt;
    } state_;

    struct Slots {
        rclcpp::Subscription<truck_msgs::msg::Trajectory>::SharedPtr trajectory = nullptr;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry = nullptr;
    } slot_;

    struct Signals {
        rclcpp::Publisher<truck_msgs::msg::Control>::SharedPtr command = nullptr;
    } signal_;

    std::chrono::duration<double> timeout_{0.20};
    std::unique_ptr<PurePursuit> controller_ = nullptr;
};

}  // namespace truck::pure_pursuit