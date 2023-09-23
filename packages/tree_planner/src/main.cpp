#include "tree_planner/tree_planner_node.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::planner::visualization::TreePlannerNode>());
    rclcpp::shutdown();
    return 0;
}
