#include "planner/planner_node.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::planner::node::PlannerNode>());
    rclcpp::shutdown();
    return 0;
}