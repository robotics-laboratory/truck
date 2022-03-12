#pragma once

#include "planning_interfaces/msg/point.hpp"
#include "pure_pursuit_msgs/msg/state.hpp"
#include "pure_pursuit_msgs/msg/command.hpp"

#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <optional>

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
    std::optional<pure_pursuit_msgs::msg::Command> get_motion(
          const pure_pursuit_msgs::msg::State &state
        , const std::vector<planning_interfaces::msg::Point> &path
    );
};

};
