#pragma once
#include "planning_interfaces/msg/path.hpp"
#include "planning_interfaces/msg/point.hpp"
#include "planning_interfaces/msg/scene.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shared_state.hpp"

#include <memory>
#include <thread>


namespace planning_node {

using namespace planning_interfaces;

std::thread start_planner(
    rclcpp::Node* node,
    SharedState::SharedPtr shared_state,
    std::string config_path,
    rclcpp::Logger logger
);

}
