#include "planner/visualization.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::planner::visualization::PlannerVisualizationNode>());
    rclcpp::shutdown();
    return 0;
}