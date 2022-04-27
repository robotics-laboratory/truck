#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pure_pursuit_msgs/msg/command.hpp"

#include "visual_info.hpp"

#include "rclcpp/rclcpp.hpp"

#include <vector>

namespace pure_pursuit {

struct Parameters {
    Parameters(rclcpp::Node &node)
    : max_velocity(node.declare_parameter<double>("max_velocity", 1))
    , max_accel(node.declare_parameter<double>("max_accel", 1))
    , lookahead_distance(node.declare_parameter<double>("lookahead_distance", 1))
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
        , VisualInfo *visual_info = nullptr
    );
};

};
