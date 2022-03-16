#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pure_pursuit_msgs/msg/command.hpp"

#include "rclcpp/rclcpp.hpp"

#include <vector>

namespace pure_pursuit {

struct Parameters {
    Parameters(const rclcpp::Node &node)
    : max_velocity(node.get_parameter("max_velocity").get_value<double>())
    , max_accel(node.get_parameter("max_accel").get_value<double>())
    , lookahead_distance(node.get_parameter("lookahead_distance").get_value<double>())
    {}

    double max_velocity;
    double max_accel;
    double lookahead_distance;
};

class Controller {
private:
    Parameters params;
public:
    Controller(const Parameters &params): params{params} {}
    pure_pursuit_msgs::msg::Command get_motion(
          const nav_msgs::msg::Odometry &odometry
        , const std::vector<geometry_msgs::msg::PoseStamped> &path
    );
};

};
