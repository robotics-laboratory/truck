#include "routing_planner/planner_node.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::routing_planner::node::PlannerNode>());
    rclcpp::shutdown();
    return 0;
}