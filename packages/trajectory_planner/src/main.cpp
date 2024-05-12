#include "trajectory_planner/node.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<truck::trajectory_planner::visualization::TrajectoryPlannerNode>());
    rclcpp::shutdown();
    return 0;
}